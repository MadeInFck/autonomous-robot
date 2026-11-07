"""Tests for the Flask web server: Basic Auth, scrypt cache, CSRF header,
waypoint record guard and fail-closed secrets loading.

The classes suffixed *Attack* replay, end to end, the weaknesses found during
the pentest: credential brute-force, scrypt-DoS, username timing oracle, NaN
injection on /api/move and thread exhaustion via /api/stream and
/api/waypoints/record.
"""

import base64
import itertools
import os
import sys
import threading
import time
import types

import pytest
from unittest.mock import MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Patch gpiod before importing main (motor_controller handles ImportError,
# but a mock keeps behavior identical across machines)
import unittest.mock
sys.modules.setdefault('gpiod', unittest.mock.MagicMock())
sys.modules.setdefault('gpiod.line', unittest.mock.MagicMock())

from werkzeug.security import generate_password_hash

import telemetry.web_server as web_server_module
from telemetry.web_server import WebServer

USERNAME = 'robot'
PASSWORD = 'correct-horse'
# pbkdf2 with few iterations: fast for tests (production uses scrypt)
PASSWORD_HASH = generate_password_hash(PASSWORD, method='pbkdf2:sha256:1000')

CSRF = {'X-Robot-Api': '1'}


def basic_auth(user, password):
    token = base64.b64encode(f"{user}:{password}".encode()).decode()
    return {'Authorization': f'Basic {token}'}


@pytest.fixture
def server():
    return WebServer(auth_username=USERNAME, auth_password_hash=PASSWORD_HASH)


@pytest.fixture
def client(server):
    return server.app.test_client()


class TestBasicAuth:
    def test_no_credentials_rejected(self, client):
        assert client.get('/api/status').status_code == 401

    def test_wrong_password_rejected(self, client):
        r = client.get('/api/status', headers=basic_auth(USERNAME, 'wrong'))
        assert r.status_code == 401

    def test_wrong_username_rejected(self, client):
        r = client.get('/api/status', headers=basic_auth('nope', PASSWORD))
        assert r.status_code == 401

    def test_valid_credentials_accepted(self, client):
        r = client.get('/api/status', headers=basic_auth(USERNAME, PASSWORD))
        assert r.status_code == 200

    def test_no_auth_configured_allows_get(self):
        client = WebServer().app.test_client()
        assert client.get('/api/status').status_code == 200

    def test_index_renders_template(self, client):
        r = client.get('/', headers=basic_auth(USERNAME, PASSWORD))
        assert r.status_code == 200
        assert b'Robot Mecanum' in r.data
        assert b'apiFetch' in r.data


class TestAuthCache:
    def test_scrypt_computed_once(self, client, monkeypatch):
        # First request: slow hash verified then cached
        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, PASSWORD)).status_code == 200

        # Any further call to the slow hash would fail loudly
        def boom(*args, **kwargs):
            raise AssertionError('check_password_hash re-computed despite cache')
        monkeypatch.setattr(web_server_module, 'check_password_hash', boom)

        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, PASSWORD)).status_code == 200

    def test_cache_does_not_accept_other_password(self, client):
        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, PASSWORD)).status_code == 200
        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, 'other')).status_code == 401


class TestCsrfHeader:
    def test_post_without_header_rejected(self, client):
        r = client.post('/api/stop', headers=basic_auth(USERNAME, PASSWORD))
        assert r.status_code == 403

    def test_post_with_header_accepted(self, client):
        r = client.post('/api/stop',
                        headers={**basic_auth(USERNAME, PASSWORD), **CSRF})
        assert r.status_code == 200

    def test_delete_without_header_rejected(self, client):
        r = client.delete('/api/waypoints/0',
                          headers=basic_auth(USERNAME, PASSWORD))
        assert r.status_code == 403

    def test_get_does_not_require_header(self, client):
        r = client.get('/api/status', headers=basic_auth(USERNAME, PASSWORD))
        assert r.status_code == 200

    def test_header_required_even_without_auth(self):
        client = WebServer().app.test_client()
        assert client.post('/api/stop').status_code == 403
        assert client.post('/api/stop', headers=CSRF).status_code == 200

    def test_auth_still_required_with_header(self, client):
        assert client.post('/api/stop', headers=CSRF).status_code == 401


class TestBruteForceAttack:
    """Attack: hammer the login with wrong passwords.

    Each wrong password used to force a full scrypt (32 MB, ~60 ms on the Pi5),
    so a flood could also monopolize the cheroot thread pool.
    """

    def _fail(self, client, n):
        for _ in range(n):
            yield client.get('/api/status',
                             headers=basic_auth(USERNAME, 'wrong')).status_code

    def test_throttled_after_budget(self, client):
        codes = list(self._fail(client, web_server_module._AUTH_MAX_FAILURES))
        assert codes == [401] * web_server_module._AUTH_MAX_FAILURES
        r = client.get('/api/status', headers=basic_auth(USERNAME, 'wrong'))
        assert r.status_code == 429
        assert r.headers['Retry-After'] == str(int(web_server_module._AUTH_WINDOW_S))

    def test_throttle_skips_the_kdf(self, client, monkeypatch):
        """The DoS angle: once throttled, no scrypt runs at all."""
        list(self._fail(client, web_server_module._AUTH_MAX_FAILURES))

        def boom(*args, **kwargs):
            raise AssertionError('KDF executed while throttled (DoS still open)')
        monkeypatch.setattr(web_server_module, 'check_password_hash', boom)

        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, 'wrong')).status_code == 429

    def test_throttle_also_blocks_valid_credentials(self, client):
        """A tripped IP is refused outright — no oracle on the throttle either."""
        list(self._fail(client, web_server_module._AUTH_MAX_FAILURES))
        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, PASSWORD)).status_code == 429

    def test_window_expiry_restores_access(self, server, client):
        list(self._fail(client, web_server_module._AUTH_MAX_FAILURES))
        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, PASSWORD)).status_code == 429
        # Age every recorded failure past the window
        with server._auth_fail_lock:
            for fails in server._auth_failures.values():
                for i in range(len(fails)):
                    fails[i] -= web_server_module._AUTH_WINDOW_S + 1
        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, PASSWORD)).status_code == 200


class TestThrottleSparesLegitimateUse:
    """Manual piloting must never be throttled: it always authenticates."""

    def test_high_frequency_piloting_never_throttled(self, client):
        codes = set()
        for _ in range(web_server_module._AUTH_MAX_FAILURES * 6):
            r = client.post('/api/move', json={'vx': 0, 'vy': 1, 'omega': 0},
                            headers={**basic_auth(USERNAME, PASSWORD), **CSRF})
            codes.add(r.status_code)
        assert codes == {200}

    def test_credential_less_requests_are_not_counted(self, client):
        """A browser's first hit carries no credentials — it must not count."""
        for _ in range(web_server_module._AUTH_MAX_FAILURES + 5):
            assert client.get('/api/status').status_code == 401
        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, PASSWORD)).status_code == 200

    def test_piloting_survives_someone_elses_failed_attempts(self, server, client):
        """Failures are tracked per IP, so another host can't lock the pilot out."""
        for _ in range(web_server_module._AUTH_MAX_FAILURES + 2):
            client.get('/api/status', headers=basic_auth(USERNAME, 'wrong'),
                       environ_overrides={'REMOTE_ADDR': '192.168.4.99'})
        assert client.get('/api/status',
                          headers=basic_auth(USERNAME, PASSWORD)).status_code == 200


class TestUsernameTimingOracleAttack:
    """Attack: measure response time to tell a valid username from an invalid
    one. Fixed by always running the KDF, whatever the username."""

    def test_kdf_runs_on_unknown_username(self, client, monkeypatch):
        tried = []
        monkeypatch.setattr(web_server_module, 'check_password_hash',
                            lambda h, p: tried.append(p) or False)
        assert client.get('/api/status',
                          headers=basic_auth('inconnu', PASSWORD)).status_code == 401
        assert tried, 'no KDF on an unknown username → timing oracle'

    def test_unknown_username_never_authenticates(self, client):
        assert client.get('/api/status',
                          headers=basic_auth('inconnu', PASSWORD)).status_code == 401
        # and the failed attempt must not have poisoned the fast-path cache
        assert client.get('/api/status',
                          headers=basic_auth('inconnu', PASSWORD)).status_code == 401


class TestMoveInjectionAttack:
    """Attack: {"vx": NaN} used to pass min/max untouched and reach the mixer
    as a full-speed command."""

    @staticmethod
    def _server():
        motors = MagicMock()
        motors.get_status.return_value = {}  # JSON-serializable telemetry
        return WebServer(motor_controller=motors), motors

    @pytest.mark.parametrize('payload', [
        '{"vx": NaN}',
        '{"vy": Infinity}',
        '{"omega": -Infinity}',
        '{"vx": 0, "vy": 0, "omega": 0, "speed": NaN}',
    ])
    def test_non_finite_rejected(self, payload):
        server, motors = self._server()
        client = server.app.test_client()
        r = client.post('/api/move', data=payload,
                        content_type='application/json', headers=CSRF)
        assert r.status_code == 400
        motors.move.assert_not_called()

    def test_valid_move_still_accepted_and_clamped(self):
        server, motors = self._server()
        client = server.app.test_client()
        r = client.post('/api/move',
                        json={'vx': 5.0, 'vy': -5.0, 'omega': 0.5, 'speed': 400},
                        headers=CSRF)
        assert r.status_code == 200
        motors.move.assert_called_once_with(1.0, -1.0, 0.5, 100)


class TestStreamExhaustionAttack:
    """Attack: open more MJPEG streams than cheroot has threads, freezing the
    UI (and the emergency STOP) for everyone."""

    @staticmethod
    def _camera(started, release):
        camera = MagicMock()
        camera.has_error.return_value = False

        def gen():
            started.release()
            yield b'frame'
            release.wait(5)
        camera.generate_stream.side_effect = gen
        return camera

    def test_extra_streams_refused_and_slots_reused(self):
        cap = web_server_module._MAX_STREAM_CLIENTS
        started, release = threading.Semaphore(0), threading.Event()
        server = WebServer(camera=self._camera(started, release))
        codes = []

        def hold():
            r = server.app.test_client().get('/api/stream')
            codes.append(r.status_code)
            r.get_data()  # stays "connected" until the gate opens
            r.close()     # what the WSGI server does when the client leaves

        threads = [threading.Thread(target=hold, daemon=True) for _ in range(cap)]
        try:
            for t in threads:
                t.start()
            for _ in threads:
                assert started.acquire(timeout=5), 'stream never started'
            # Pool saturated: the next client is refused instead of pinning a thread
            extra = server.app.test_client().get('/api/stream')
            assert extra.status_code == 503
            assert extra.headers['Retry-After'] == '5'
        finally:
            release.set()
            for t in threads:
                t.join(timeout=5)

        assert codes == [200] * cap
        assert server._stream_clients == 0, 'slot leaked on disconnect'
        # Slots freed on disconnect: a new client is served again
        again = server.app.test_client().get('/api/stream')
        assert again.status_code == 200
        again.close()

    def test_refusal_does_not_consume_a_slot(self):
        camera = MagicMock()
        camera.has_error.return_value = True
        server = WebServer(camera=camera)
        client = server.app.test_client()
        for _ in range(web_server_module._MAX_STREAM_CLIENTS * 3):
            assert client.get('/api/stream').status_code == 503
        assert server._stream_clients == 0


class TestRecordExhaustionAttack:
    """Attack: /api/waypoints/record blocks its thread ~10 s; repeating it
    used to pin one more thread per call."""

    @pytest.fixture
    def fast_sleep(self, monkeypatch):
        """Neutralize the 1 Hz GPS pacing inside the web server only."""
        real = time
        shim = types.SimpleNamespace(time=real.time, sleep=lambda s: None)
        monkeypatch.setattr(web_server_module, 'time', shim)

    def test_concurrent_record_refused(self, fast_sleep):
        entered, gate = threading.Semaphore(0), threading.Event()
        counter = itertools.count()
        def get_last_data():
            if next(counter) == 0:      # first sampler: hold the critical section
                entered.release()
                gate.wait(5)
            return types.SimpleNamespace(has_fix=True, latitude=48.8566,
                                         longitude=2.3522, timestamp=time.time())

        sensors = MagicMock()
        sensors.get_last_data.side_effect = get_last_data
        patrol = MagicMock()
        patrol.record_waypoint.return_value = 0
        server = WebServer(patrol_manager=patrol, sensor_receiver=sensors)
        first = []

        def record():
            first.append(server.app.test_client()
                         .post('/api/waypoints/record', headers=CSRF).status_code)

        t = threading.Thread(target=record, daemon=True)
        try:
            t.start()
            assert entered.acquire(timeout=5), 'first record never started'
            r = server.app.test_client().post('/api/waypoints/record', headers=CSRF)
            assert r.status_code == 409
        finally:
            gate.set()
            t.join(timeout=5)

        assert first == [200]
        # Lock released: recording is possible again right after
        assert server.app.test_client().post(
            '/api/waypoints/record', headers=CSRF).status_code == 200

    def test_rejected_while_patrolling(self):
        pilot = MagicMock()
        pilot.is_active.return_value = True
        server = WebServer(pilot=pilot, patrol_manager=MagicMock(),
                           sensor_receiver=MagicMock())
        client = server.app.test_client()
        r = client.post('/api/waypoints/record', headers=CSRF)
        assert r.status_code == 409


class TestPilotHeartbeatWatchdog:
    """Safety (not network security): if the pilot's control thread dies while
    navigating, the manual watchdog is bypassed and the motors would hold their
    last command. The heartbeat closes that gap."""

    @staticmethod
    def _run_watchdog(server, duration=0.35):
        server._running = True
        t = threading.Thread(target=server._watchdog_loop, daemon=True)
        t.start()
        time.sleep(duration)
        server._running = False
        t.join(timeout=2)

    def test_dead_control_thread_stops_motors(self):
        motors = MagicMock()
        pilot = MagicMock()
        pilot.is_active.return_value = True
        pilot.control_heartbeat_age.return_value = 99.0
        server = WebServer(motor_controller=motors, pilot=pilot)
        self._run_watchdog(server)
        assert motors.stop.called, 'runaway: motors kept their last command'
        assert pilot.stop.called, 'patrol left in NAVIGATING with a dead thread'

    def test_healthy_pilot_is_left_alone(self):
        motors = MagicMock()
        pilot = MagicMock()
        pilot.is_active.return_value = True
        pilot.control_heartbeat_age.return_value = 0.2
        server = WebServer(motor_controller=motors, pilot=pilot)
        self._run_watchdog(server)
        motors.stop.assert_not_called()
        pilot.stop.assert_not_called()

    def test_pilot_without_heartbeat_api_never_trips(self):
        pilot = MagicMock()  # control_heartbeat_age returns a MagicMock
        server = WebServer(pilot=pilot)
        assert server._pilot_heartbeat_stale() is False
        server.pilot = None
        assert server._pilot_heartbeat_stale() is False

    def test_manual_watchdog_still_stops_motors(self):
        motors = MagicMock()
        server = WebServer(motor_controller=motors)
        server._last_move_time = time.time() - 10
        self._run_watchdog(server)
        assert motors.stop.called

    def test_joystick_loss_logged_once_when_moving(self, caplog):
        """A cut that stopped a moving robot is a felt jerk: worth a warning."""
        motors = MagicMock()
        motors.get_status.return_value = {'FL': {'speed': 80}, 'FR': {'speed': 80}}
        server = WebServer(motor_controller=motors)
        server._last_move_time = time.time() - 10
        with caplog.at_level('WARNING'):
            self._run_watchdog(server)
        assert len([r for r in caplog.records if 'Joystick muet' in r.message]) == 1

    def test_idle_robot_not_logged(self, caplog):
        motors = MagicMock()
        motors.get_status.return_value = {'FL': {'speed': 0}}
        server = WebServer(motor_controller=motors)
        with caplog.at_level('WARNING'):
            self._run_watchdog(server)
        assert not any('Joystick muet' in r.message for r in caplog.records)


class TestRecordFreshness:
    """A frozen UART link (Pico dead) must not be recorded as a waypoint."""

    def test_stale_frames_rejected(self, monkeypatch):
        shim = types.SimpleNamespace(time=time.time, sleep=lambda s: None)
        monkeypatch.setattr(web_server_module, 'time', shim)
        sensors = MagicMock()
        sensors.get_last_data.return_value = types.SimpleNamespace(
            has_fix=True, latitude=48.8566, longitude=2.3522,
            timestamp=time.time() - 30)
        patrol = MagicMock()
        server = WebServer(patrol_manager=patrol, sensor_receiver=sensors)
        r = server.app.test_client().post('/api/waypoints/record', headers=CSRF)
        assert r.status_code == 400
        patrol.record_waypoint.assert_not_called()


class TestManualControlVsPilot:
    def _server(self, active):
        motors = MagicMock()
        motors.get_status.return_value = {}
        pilot = MagicMock()
        pilot.is_active.return_value = active
        return WebServer(motor_controller=motors, pilot=pilot), motors

    def test_joystick_ignored_while_patrolling(self):
        server, motors = self._server(active=True)
        r = server.app.test_client().post('/api/move', json={'vy': 1}, headers=CSRF)
        assert r.status_code == 200
        motors.move.assert_not_called()

    def test_joystick_works_once_patrol_finished(self):
        """PATROL_COMPLETE (thread ended) used to keep the joystick locked."""
        server, motors = self._server(active=False)
        server.app.test_client().post('/api/move', json={'vy': 1}, headers=CSRF)
        motors.move.assert_called_once()


class TestLidarOverlay:
    """/api/lidar carries what the radar draws: obstacles + avoidance decision."""

    def _get(self, active=False):
        import numpy as np
        from lidar.data import Scan, ScanPoint
        from lidar.detection.obstacles import ObstacleDetector
        from navigation.obstacle_avoider import ObstacleAvoider

        # A wall 0.6 m ahead: the corridor is blocked, the sides are free
        xs = np.arange(-300, 301, 20, dtype=float)
        scan = Scan(points=[ScanPoint(float(np.degrees(np.arctan2(x, 600)) % 360),
                                      float(np.hypot(x, 600)), 40) for x in xs],
                    timestamp=time.time())
        detection = ObstacleDetector().detect(scan)
        avoider = ObstacleAvoider(obstacle_distance_mm=800)

        lidar = MagicMock()
        lidar.get_last_scan.return_value = scan
        pilot = MagicMock()
        pilot.is_active.return_value = active
        pilot.avoider = avoider
        pilot.obstacle_view.return_value = (detection, avoider.evaluate(detection, 0))
        server = WebServer(lidar_scanner=lidar, pilot=pilot)
        return server.app.test_client().get('/api/lidar').get_json()

    def test_wall_and_avoidance(self):
        data = self._get()
        view = data['view']
        assert view['preview'] is True
        assert view['corridor'] == {'half_angle': 30, 'distance_mm': 800}
        [wall] = view['obstacles']
        assert wall['type'] == 'wall' and len(wall['seg']) == 4
        assert view['avoidance']['status'] == 'avoiding'
        assert abs(view['avoidance']['turn_deg']) > 30   # out of the wall, not into it
        assert data['nearest_mm'] == 600

    def test_far_obstacles_not_drawn(self):
        """Beyond the avoidance distance an obstacle changes nothing."""
        from lidar.data import Scan
        from lidar.detection.types import DetectionResult
        from navigation.obstacle_avoider import ObstacleAvoider
        from tests.test_obstacle_avoider import make_obstacle

        avoider = ObstacleAvoider(obstacle_distance_mm=800)
        detection = DetectionResult(obstacles=[make_obstacle(90, 500), make_obstacle(90, 1200)])
        pilot = MagicMock()
        pilot.is_active.return_value = False
        pilot.avoider = avoider
        pilot.obstacle_view.return_value = (detection, avoider.evaluate(detection, 0))
        lidar = MagicMock()
        lidar.get_last_scan.return_value = Scan(timestamp=time.time())
        view = WebServer(lidar_scanner=lidar, pilot=pilot).app.test_client().get('/api/lidar').get_json()['view']
        [near] = view['obstacles']
        assert near['x'] == 500

    def test_not_preview_while_patrolling(self):
        assert self._get(active=True)['view']['preview'] is False

    def test_no_pilot_no_overlay(self):
        lidar = MagicMock()
        lidar.get_last_scan.return_value = None
        data = WebServer(lidar_scanner=lidar).app.test_client().get('/api/lidar').get_json()
        assert data['points'] == [] and data['nearest_mm'] is None


class TestLidarSwitch:
    def _client(self, running=True, patrolling=False):
        lidar = MagicMock()
        lidar.is_running = running
        lidar.start.side_effect = lambda: setattr(lidar, 'is_running', True)
        lidar.stop.side_effect = lambda: setattr(lidar, 'is_running', False)
        lidar.get_last_scan.return_value = None
        pilot = MagicMock()
        pilot.is_active.return_value = patrolling
        server = WebServer(lidar_scanner=lidar, pilot=pilot)
        return server.app.test_client(), lidar, pilot

    def test_switch_off_and_on(self):
        client, lidar, _ = self._client()
        r = client.post('/api/lidar/enable', json={'enabled': False}, headers=CSRF)
        assert r.status_code == 200 and r.get_json()['running'] is False
        assert client.get('/api/status').get_json()['lidar_running'] is False
        r = client.post('/api/lidar/enable', json={'enabled': True}, headers=CSRF)
        assert r.get_json()['running'] is True

    def test_cannot_blind_a_running_patrol(self):
        client, lidar, _ = self._client(patrolling=True)
        r = client.post('/api/lidar/enable', json={'enabled': False}, headers=CSRF)
        assert r.status_code == 409
        lidar.stop.assert_not_called()

    def test_no_patrol_with_lidar_off(self):
        client, _, pilot = self._client(running=False)
        r = client.post('/api/patrol/start', headers=CSRF)
        assert r.status_code == 409 and 'LiDAR' in r.get_json()['message']
        pilot.start.assert_not_called()

    def test_patrol_starts_with_lidar_on(self):
        client, _, pilot = self._client()
        assert client.post('/api/patrol/start', headers=CSRF).status_code == 200
        pilot.start.assert_called_once()

    def test_bad_payload(self):
        client, lidar, _ = self._client()
        r = client.post('/api/lidar/enable', json={'enabled': 'no'}, headers=CSRF)
        assert r.status_code == 400
        lidar.stop.assert_not_called()

    def test_start_failure_reported(self):
        client, lidar, _ = self._client(running=False)
        lidar.start.side_effect = OSError('port occupe')
        r = client.post('/api/lidar/enable', json={'enabled': True}, headers=CSRF)
        assert r.status_code == 500 and 'port occupe' in r.get_json()['message']


class TestMotorTrimConfig:
    """robot_config.yaml `motors: trim:` -> MecanumController factors."""

    @staticmethod
    def _trim(trim):
        import main
        return main._build_motor_trim({'motors': {'trim': trim}})

    def test_valid_values_kept(self):
        assert self._trim({'FL': 0.95, 'RL': 1}) == {'FL': 0.95, 'RL': 1.0}

    def test_out_of_range_and_unknown_ignored(self, caplog):
        with caplog.at_level('WARNING'):
            trim = self._trim({'FL': 1.2, 'FR': 0.1, 'XX': 0.9, 'RR': 'fast', 'RL': 0.9})
        assert trim == {'RL': 0.9}
        assert len([r for r in caplog.records if 'motors.trim' in r.message]) == 4

    def test_absent_section(self):
        import main
        assert main._build_motor_trim({}) == {}
        assert main._build_motor_trim({'motors': {'trim': None}}) == {}


class TestLoadWebConfig:
    """main.load_web_config must be fail-closed: any problem raises."""

    @staticmethod
    def _load():
        import main
        return main.load_web_config

    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(RuntimeError, match='introuvable'):
            self._load()(str(tmp_path / 'absent.yaml'))

    def test_invalid_yaml_raises(self, tmp_path):
        p = tmp_path / 'secrets.yaml'
        p.write_text('auth: [invalid\n')
        with pytest.raises(RuntimeError, match='illisible'):
            self._load()(str(p))

    def test_missing_auth_keys_raises(self, tmp_path):
        p = tmp_path / 'secrets.yaml'
        p.write_text('auth:\n  username: bob\n')
        with pytest.raises(RuntimeError, match='incomplete'):
            self._load()(str(p))

    def test_valid_config(self, tmp_path):
        p = tmp_path / 'secrets.yaml'
        p.write_text('auth:\n  username: bob\n  password_hash: "hash"\n')
        user, pw_hash, cert, key = self._load()(str(p))
        assert user == 'bob'
        assert pw_hash == 'hash'
        assert cert is None and key is None

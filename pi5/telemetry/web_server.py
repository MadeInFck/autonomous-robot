"""
Flask web server for Mecanum robot control via smartphone
Dual touch joystick interface + real-time telemetry
+ 360deg lidar view + waypoint management + autonomous patrol
"""

import hashlib
import hmac
import logging
import math
import subprocess
import time
import socket
import threading
from collections import deque
from flask import Flask, render_template, jsonify, request, Response
from werkzeug.security import check_password_hash
from dataclasses import asdict

# Brute-force / scrypt-DoS throttle: after this many FAILED auth attempts from
# one IP within the window, further attempts are refused (429) without running
# the costly KDF. Only failures count — a client sending valid credentials
# (cached after the first success) is never throttled, so high-frequency manual
# piloting is unaffected.
_AUTH_MAX_FAILURES = 10
_AUTH_WINDOW_S = 60.0

# Thread-exhaustion guards. cheroot serves with a fixed pool (numthreads=12) and
# an MJPEG stream holds its thread for as long as the client stays connected:
# without a cap, a few tabs left open would starve the pool and freeze the UI —
# including the emergency STOP button.
_MAX_STREAM_CLIENTS = 3

# Safety: while the pilot owns the motors the manual move-watchdog is bypassed,
# so a dead control thread would leave the last command applied indefinitely.
# The pilot feeds a heartbeat each cycle (~5 Hz); this is the staleness budget.
_PILOT_HEARTBEAT_TIMEOUT_S = 3.0

# Production WSGI server: single process (the app shares live objects with the
# motor/UART threads), multi-threaded, TLS via ssl_adapter. Falls back to the
# werkzeug dev server if not installed.
try:
    from cheroot import wsgi as _cheroot_wsgi
    from cheroot.ssl.builtin import BuiltinSSLAdapter as _BuiltinSSLAdapter
except ImportError:
    _cheroot_wsgi = None

logger = logging.getLogger(__name__)


def get_local_ip():
    """Detects the local IP address"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return 'localhost'




class WebServer:
    """Flask web server for robot control"""

    def __init__(self, motor_controller=None, sensor_receiver=None,
                 lidar_scanner=None, patrol_manager=None, pilot=None,
                 camera=None,
                 host='0.0.0.0', port=8085,
                 auth_username=None, auth_password_hash=None,
                 ssl_cert=None, ssl_key=None,
                 patrol_file=None):
        self.app = Flask(__name__)
        self.app.config['MAX_CONTENT_LENGTH'] = 16 * 1024  # 16 KB max
        self.motor_controller = motor_controller
        self.sensor_receiver = sensor_receiver
        self.lidar_scanner = lidar_scanner
        self.patrol_manager = patrol_manager
        self.pilot = pilot
        self.camera = camera
        self.host = host
        self.port = port
        self._patrol_file = patrol_file
        self._thread = None
        self._running = False
        self._last_move_time = 0.0
        self._watchdog_interval = 0.5  # seconds: stop motors if no command received
        self._auth_username = auth_username
        self._auth_password_hash = auth_password_hash
        # sha256("user:password") of the last successfully verified credentials.
        # Avoids re-running scrypt (~50-100 ms on Pi5) on every polled request.
        self._auth_ok_digest = None
        # Per-IP failed-auth tracking for the throttle (ip -> deque[timestamps]).
        self._auth_failures = {}
        self._auth_fail_lock = threading.Lock()
        # Single-flight guard for the blocking waypoint-record sampling.
        self._record_lock = threading.Lock()
        # Concurrent MJPEG stream accounting (see _MAX_STREAM_CLIENTS).
        self._stream_clients = 0
        self._stream_lock = threading.Lock()
        self._ssl_context = (ssl_cert, ssl_key) if ssl_cert and ssl_key else None
        self._server = None  # cheroot server instance (None = werkzeug fallback)
        self._register_routes()

    def _is_auth_throttled(self, ip: str) -> bool:
        """True if this IP has exceeded the failed-auth budget in the window."""
        now = time.time()
        with self._auth_fail_lock:
            fails = self._auth_failures.get(ip)
            if not fails:
                return False
            while fails and now - fails[0] > _AUTH_WINDOW_S:
                fails.popleft()
            if not fails:
                del self._auth_failures[ip]
                return False
            return len(fails) >= _AUTH_MAX_FAILURES

    def _record_auth_failure(self, ip: str):
        """Records one failed auth attempt from ip (called only on wrong creds)."""
        now = time.time()
        with self._auth_fail_lock:
            fails = self._auth_failures.get(ip)
            if fails is None:
                fails = deque()
                self._auth_failures[ip] = fails
            while fails and now - fails[0] > _AUTH_WINDOW_S:
                fails.popleft()
            fails.append(now)
            # Bound memory against a spoofed-IP flood (unrealistic on the AP,
            # but keeps the dict from growing without limit).
            if len(self._auth_failures) > 256:
                self._auth_failures.pop(next(iter(self._auth_failures)))

    def _check_credentials(self, username: str, password: str) -> bool:
        """Verifies Basic Auth credentials (constant-time comparisons)."""
        digest = hashlib.sha256(f"{username}:{password}".encode()).digest()
        if self._auth_ok_digest is not None and \
           hmac.compare_digest(digest, self._auth_ok_digest):
            return True
        user_ok = hmac.compare_digest(username.encode(), self._auth_username.encode())
        # Always run the KDF, even when the username is wrong, so response time
        # does not reveal whether the username exists (closes a timing oracle).
        pw_ok = check_password_hash(self._auth_password_hash, password)
        if user_ok and pw_ok:
            self._auth_ok_digest = digest
            return True
        return False

    def _acquire_stream_slot(self) -> bool:
        """Reserves one of the _MAX_STREAM_CLIENTS streaming slots."""
        with self._stream_lock:
            if self._stream_clients >= _MAX_STREAM_CLIENTS:
                return False
            self._stream_clients += 1
            return True

    def _release_stream_slot(self):
        with self._stream_lock:
            self._stream_clients = max(0, self._stream_clients - 1)

    def _pilot_heartbeat_stale(self) -> bool:
        """True if the pilot's control thread stopped feeding its heartbeat.

        Tolerant by design: a pilot without the heartbeat API (or a mock)
        never reports stale, so the watchdog can't stop the robot spuriously.
        """
        getter = getattr(self.pilot, 'control_heartbeat_age', None)
        if getter is None:
            return False
        try:
            return float(getter()) > _PILOT_HEARTBEAT_TIMEOUT_S
        except (TypeError, ValueError):
            return False

    def _register_routes(self):

        # CSRF protection + auth on all requests (before_request hook)
        @self.app.before_request
        def check_auth():
            # State-changing methods must carry the custom header set by the
            # UI's apiFetch(). A cross-site form/fetch can't add it without a
            # CORS preflight (which is denied), blocking CSRF even though the
            # browser auto-attaches the memorized Basic Auth credentials.
            if request.method not in ('GET', 'HEAD', 'OPTIONS') \
               and request.headers.get('X-Robot-Api') != '1':
                return Response('En-tete X-Robot-Api requis', 403)
            if not self._auth_username or not self._auth_password_hash:
                return None
            ip = request.remote_addr or 'unknown'
            # Refuse before the costly KDF if this IP is over its failure budget.
            if self._is_auth_throttled(ip):
                return Response(
                    'Trop de tentatives, reessayez plus tard', 429,
                    {'Retry-After': str(int(_AUTH_WINDOW_S))}
                )
            auth = request.authorization
            if auth and auth.username and auth.password:
                if self._check_credentials(auth.username, auth.password):
                    return None
                # Credentials were provided but wrong: count it (a request with
                # no credentials at all — e.g. a browser's first hit — is not
                # counted, so normal page loads never trip the throttle).
                self._record_auth_failure(ip)
            return Response(
                'Authentification requise', 401,
                {'WWW-Authenticate': 'Basic realm="Robot Mecanum"'}
            )

        # --- Existing routes ---

        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.app.route('/api/move', methods=['POST'])
        def api_move():
            try:
                data = request.get_json()
                vx = float(data.get('vx', 0))
                vy = float(data.get('vy', 0))
                omega = float(data.get('omega', 0))
                speed = float(data.get('speed', 50))
                # Reject non-finite input explicitly: min/max would let NaN slip
                # through to the motor mixer as a spurious full-speed command.
                if not all(math.isfinite(v) for v in (vx, vy, omega, speed)):
                    return jsonify({'status': 'error', 'message': 'Valeur non finie'}), 400
                vx = max(-1.0, min(1.0, vx))
                vy = max(-1.0, min(1.0, vy))
                omega = max(-1.0, min(1.0, omega))
                speed = max(0, min(100, int(speed)))
                self._last_move_time = time.time()

                motors = None
                pilot_active = (
                    self.pilot and
                    self.pilot.get_status().get('state') not in ('IDLE',)
                )
                if self.motor_controller and not pilot_active:
                    self.motor_controller.move(vx, vy, omega, speed)
                    motors = self.motor_controller.get_status()

                return jsonify({'status': 'ok', 'motors': motors})
            except Exception:
                return jsonify({'status': 'error', 'message': 'Requete invalide'}), 400

        @self.app.route('/api/sensors')
        def api_sensors():
            if self.sensor_receiver:
                data = self.sensor_receiver.get_last_data()
                if data:
                    return jsonify(asdict(data))
            return jsonify({})

        @self.app.route('/api/status')
        def api_status():
            uart_ok = False
            if self.sensor_receiver:
                data = self.sensor_receiver.get_last_data()
                uart_ok = data is not None and (time.time() - data.timestamp) < 3.0
            return jsonify({
                'motors_connected': self.motor_controller is not None,
                'sensors_connected': self.sensor_receiver is not None,
                'lidar_connected': self.lidar_scanner is not None,
                'patrol_available': self.patrol_manager is not None,
                'camera_connected': self.camera is not None and not self.camera.has_error(),
                'uart_ok': uart_ok,
            })

        @self.app.route('/api/detections')
        def api_detections():
            if not self.camera:
                return jsonify({'detections': [], 'camera_ok': False, 'paused': False, 'starting': False})
            return jsonify({
                'detections': self.camera.get_recent_detections(max_age_s=60.0),
                'camera_ok': not self.camera.has_error(),
                'paused': self.camera.is_paused(),
                'starting': self.camera.is_starting(),
            })

        @self.app.route('/api/camera/toggle', methods=['POST'])
        def api_camera_toggle():
            if not self.camera:
                return jsonify({'error': 'no camera'}), 404
            if self.camera.is_paused():
                self.camera.resume()
            else:
                self.camera.pause()
            return jsonify({'paused': self.camera.is_paused()})

        @self.app.route('/api/stream')
        def api_stream():
            if not self.camera or self.camera.has_error():
                return Response('Camera non disponible', status=503)
            # Each stream pins one server thread until the client goes away.
            if not self._acquire_stream_slot():
                return Response('Trop de flux video simultanes', status=503,
                                headers={'Retry-After': '5'})
            response = Response(
                self.camera.generate_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame',
            )
            # call_on_close fires when the WSGI server closes the response —
            # normal end, client disconnect or error alike. The flag keeps a
            # double close from freeing two slots.
            released = False

            def _release():
                nonlocal released
                if not released:
                    released = True
                    self._release_stream_slot()

            response.call_on_close(_release)
            return response

        @self.app.route('/api/stop', methods=['POST'])
        def api_stop():
            if self.pilot:
                self.pilot.stop()
            if self.motor_controller:
                self.motor_controller.stop()
            return jsonify({'status': 'ok'})

        # --- Lidar routes ---

        @self.app.route('/api/lidar')
        def api_lidar():
            if not self.lidar_scanner:
                return jsonify({'points': [], 'num_points': 0, 'nearest_mm': None})

            scan = self.lidar_scanner.get_last_scan()
            if scan is None:
                return jsonify({'points': [], 'num_points': 0, 'nearest_mm': None})

            # Compact format for client: [[angle_deg, distance_mm], ...]
            points = [
                [round(p.angle_deg, 1), round(p.distance_mm)]
                for p in scan.valid_points
            ]

            nearest_mm = None
            if self.pilot:
                det = self.pilot.get_last_detection()
                if det and det.nearest_obstacle:
                    nearest_mm = round(det.nearest_obstacle.nearest_distance_mm)

            return jsonify({
                'points': points,
                'num_points': len(points),
                'nearest_mm': nearest_mm,
            })

        # --- Waypoint routes ---

        @self.app.route('/api/waypoints')
        def api_waypoints():
            if not self.patrol_manager:
                return jsonify({'waypoints': []})
            return jsonify({'waypoints': self.patrol_manager.get_waypoints()})

        @self.app.route('/api/waypoints/record', methods=['POST'])
        def api_record_waypoint():
            if not self.patrol_manager or not self.sensor_receiver:
                return jsonify({'status': 'error', 'message': 'Non disponible'}), 400
            # Recording while the robot moves would average positions along
            # the path into a meaningless midpoint.
            if self.pilot and self.pilot.get_status().get('state') != 'IDLE':
                return jsonify({'status': 'error',
                                'message': 'Arretez la patrouille avant d\'enregistrer'}), 409

            # Sampling blocks its server thread for ~10 s. Single-flight: a
            # repeated tap (or a flood) is refused instead of pinning one more
            # thread each time, which would starve the pool.
            if not self._record_lock.acquire(blocking=False):
                return jsonify({'status': 'error',
                                'message': 'Enregistrement deja en cours'}), 409
            try:
                # Average N GPS readings to reduce position error
                GPS_SAMPLES = 10
                GPS_INTERVAL = 1.0  # 1 Hz — real GPS update rate
                lats, lons = [], []
                for _ in range(GPS_SAMPLES):
                    d = self.sensor_receiver.get_last_data()
                    if d and d.has_fix:
                        lats.append(d.latitude)
                        lons.append(d.longitude)
                    time.sleep(GPS_INTERVAL)

                if len(lats) < 3:
                    return jsonify({'status': 'error', 'message': 'Pas assez de fix GPS'}), 400

                avg_lat = sum(lats) / len(lats)
                avg_lon = sum(lons) / len(lons)

                idx = self.patrol_manager.record_waypoint(avg_lat, avg_lon)
                # Auto-save waypoints
                try:
                    self.patrol_manager.save(self._patrol_file)
                except Exception as e:
                    logger.error(f"Erreur sauvegarde waypoints: {e}")
                return jsonify({'status': 'ok', 'index': idx, 'samples': len(lats)})
            finally:
                self._record_lock.release()

        @self.app.route('/api/waypoints/<int:idx>', methods=['DELETE'])
        def api_delete_waypoint(idx):
            if not self.patrol_manager:
                return jsonify({'status': 'error'}), 400
            self.patrol_manager.delete_waypoint(idx)
            try:
                self.patrol_manager.save(self._patrol_file)
            except Exception as e:
                logger.error(f"Erreur sauvegarde waypoints: {e}")
            return jsonify({'status': 'ok'})

        @self.app.route('/api/waypoints/last', methods=['DELETE'])
        def api_delete_last_waypoint():
            if not self.patrol_manager or self.patrol_manager.count == 0:
                return jsonify({'status': 'error'}), 400
            self.patrol_manager.delete_waypoint(self.patrol_manager.count - 1)
            try:
                self.patrol_manager.save(self._patrol_file)
            except Exception as e:
                logger.error(f"Erreur sauvegarde waypoints: {e}")
            return jsonify({'status': 'ok'})

        # --- System routes ---

        # Both rely on a sudoers rule restricted to exactly these commands
        # (installed by scripts/setup_pi5_env.sh in /etc/sudoers.d/robot-power).

        @self.app.route('/api/shutdown', methods=['POST'])
        def api_shutdown():
            def _do_shutdown():
                time.sleep(1)
                subprocess.run(['sudo', 'shutdown', 'now'])
            threading.Thread(target=_do_shutdown, daemon=True).start()
            return jsonify({'status': 'ok', 'message': 'Extinction en cours...'})

        @self.app.route('/api/restart', methods=['POST'])
        def api_restart():
            def _do_restart():
                time.sleep(1)
                subprocess.run(['sudo', 'reboot'])
            threading.Thread(target=_do_restart, daemon=True).start()
            return jsonify({'status': 'ok', 'message': 'Redémarrage en cours...'})

        # --- Patrol routes ---

        @self.app.route('/api/patrol/start', methods=['POST'])
        def api_patrol_start():
            if not self.pilot:
                return jsonify({'status': 'error', 'message': 'Pilote non disponible'}), 400
            self.pilot.start()
            return jsonify({'status': 'ok'})

        @self.app.route('/api/patrol/stop', methods=['POST'])
        def api_patrol_stop():
            if not self.pilot:
                return jsonify({'status': 'error'}), 400
            self.pilot.stop()
            if self.motor_controller:
                self.motor_controller.stop()
            return jsonify({'status': 'ok'})

        @self.app.route('/api/patrol/status')
        def api_patrol_status():
            if not self.pilot:
                return jsonify({'state': 'IDLE', 'waypoint_index': 0, 'waypoint_count': 0})
            return jsonify(self.pilot.get_status())

    def _watchdog_loop(self):
        """Stop motors if no move command received for watchdog_interval seconds (manual mode only)."""
        pilot_stalled = False
        while self._running:
            time.sleep(0.1)
            if not self.motor_controller:
                continue
            pilot_is_active = (
                self.pilot and
                self.pilot.get_status().get('state') not in ('IDLE',)
            )
            if pilot_is_active:
                # The manual watchdog is intentionally bypassed here (the pilot
                # drives the motors), so nothing else would catch a control
                # thread that died mid-navigation: the last command would stay
                # applied and the robot would run away. Its heartbeat does.
                if self._pilot_heartbeat_stale():
                    if not pilot_stalled:
                        logger.error(
                            "Thread pilote muet depuis >%.1fs: arret failsafe",
                            _PILOT_HEARTBEAT_TIMEOUT_S)
                        pilot_stalled = True
                    self.motor_controller.stop()
                    try:
                        # Back to IDLE: the UI shows the patrol stopped and the
                        # manual watchdog takes over holding the motors.
                        self.pilot.stop()
                    except Exception as e:
                        logger.error(f"Arret pilote impossible: {e}")
                continue
            pilot_stalled = False
            if time.time() - self._last_move_time > self._watchdog_interval:
                self.motor_controller.stop()

    def start(self, threaded=True):
        local_ip = get_local_ip()
        scheme = "https" if self._ssl_context else "http"
        url = f"{scheme}://{local_ip}:{self.port}"
        self._running = True

        if _cheroot_wsgi is not None:
            self._server = _cheroot_wsgi.Server(
                (self.host, self.port), self.app,
                numthreads=12, server_name='robot-mecanum',
            )
            if self._ssl_context:
                cert, key = self._ssl_context
                self._server.ssl_adapter = _BuiltinSSLAdapter(cert, key)
            serve = self._server.safe_start
        else:
            logger.warning("cheroot absent: serveur de dev werkzeug utilise "
                           "(pip install cheroot)")
            run_kwargs = dict(host=self.host, port=self.port, debug=False,
                              use_reloader=False, threaded=True)
            if self._ssl_context:
                run_kwargs['ssl_context'] = self._ssl_context
            serve = lambda: self.app.run(**run_kwargs)

        if threaded:
            self._thread = threading.Thread(target=serve, daemon=True)
            self._thread.start()
            threading.Thread(target=self._watchdog_loop, daemon=True).start()
            logger.info(f"Serveur web: {url}")
        else:
            logger.info(f"Serveur web: {url}")
            serve()

        return url

    def get_url(self):
        scheme = "https" if self._ssl_context else "http"
        return f"{scheme}://{get_local_ip()}:{self.port}"

    def stop(self):
        self._running = False
        if self._server is not None:
            try:
                self._server.stop()
            except Exception:
                pass


if __name__ == "__main__":
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    print("=" * 40)
    print("  Test Serveur Web - Double Joystick")
    print("=" * 40)

    motor_ctrl = None
    try:
        from motors.motor_controller import MecanumController
        motor_ctrl = MecanumController()
        print("[Motors] OK")
    except Exception as e:
        print(f"[Motors] Mode simulation (pas de GPIO): {e}")

    server = WebServer(motor_controller=motor_ctrl)
    print(f"\nOuvrir: http://localhost:8085")
    print("Ctrl+C pour arreter\n")

    try:
        server.start(threaded=False)
    except KeyboardInterrupt:
        print("\nArret")

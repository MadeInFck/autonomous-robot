"""
Serveur Web Flask pour contrôle robot Mecanum via smartphone
Interface double joystick tactile + télémétrie temps réel
"""

import json
import time
import socket
import threading
from flask import Flask, render_template_string, jsonify, request
from dataclasses import asdict


def get_local_ip():
    """Détecte l'adresse IP locale"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return 'localhost'


HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Robot Mecanum</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; touch-action: none; -webkit-user-select: none; user-select: none; }
        html { height: 100%; }
        body { font-family: -apple-system, sans-serif; background: linear-gradient(135deg, #1a1a2e, #16213e); color: #fff; min-height: 100%; display: flex; flex-direction: column; overflow: hidden; padding-top: env(safe-area-inset-top); }

        .header { text-align: center; padding: 8px; background: rgba(0,0,0,0.3); flex-shrink: 0; }
        .header h1 { font-size: 1.1em; font-weight: 500; }

        .status-bar { display: flex; justify-content: space-around; padding: 6px; background: rgba(0,0,0,0.2); font-size: 0.75em; flex-shrink: 0; }
        .status-dot { width: 8px; height: 8px; border-radius: 50%; background: #f44; display: inline-block; margin-right: 4px; vertical-align: middle; }
        .status-dot.connected { background: #4f4; }

        .sensors-panel { display: flex; justify-content: space-around; padding: 8px; background: rgba(0,0,0,0.15); font-size: 0.7em; font-family: monospace; flex-shrink: 0; }
        .sensor-group { text-align: center; }
        .sensor-label { opacity: 0.6; margin-bottom: 2px; }
        .sensor-values { display: flex; gap: 8px; }
        .sensor-axis { }
        .axis-label { opacity: 0.5; }

        .joysticks-row { display: flex; justify-content: space-around; align-items: center; flex: 1; padding: 20px; min-height: 0; }

        .joystick-wrapper { text-align: center; }
        .joystick-label { font-size: 0.85em; margin-bottom: 10px; opacity: 0.8; }
        .joystick-container { position: relative; width: 140px; height: 140px; touch-action: none; }
        .joystick-base { position: absolute; width: 140px; height: 140px; border-radius: 50%; background: rgba(255,255,255,0.1); border: 2px solid rgba(255,255,255,0.3); }
        .joystick-stick { position: absolute; width: 55px; height: 55px; border-radius: 50%; left: 42.5px; top: 42.5px; pointer-events: none; transition: transform 0.1s; }
        .joystick-stick.move { background: linear-gradient(145deg, #4a90d9, #357abd); box-shadow: 0 3px 12px rgba(74,144,217,0.5); }
        .joystick-stick.rotate { background: linear-gradient(145deg, #e94560, #c73e54); box-shadow: 0 3px 12px rgba(233,69,96,0.5); }
        .joystick-stick.active { transform: scale(1.1); }

        .bottom-bar { display: flex; justify-content: space-between; align-items: center; padding: 12px 15px; padding-bottom: calc(12px + env(safe-area-inset-bottom)); background: rgba(0,0,0,0.5); flex-shrink: 0; }

        .telemetry { font-size: 0.7em; font-family: monospace; }
        .tel-row { margin: 2px 0; }

        .speed-control { display: flex; align-items: center; gap: 8px; }
        .speed-btn { width: 38px; height: 38px; border-radius: 50%; border: 2px solid rgba(255,255,255,0.4); background: transparent; color: #fff; font-size: 1.3em; }
        .speed-btn:active { background: rgba(255,255,255,0.2); }
        .speed-value { font-size: 1.1em; min-width: 50px; text-align: center; font-weight: bold; }

        @media (min-width: 500px) {
            .joystick-container { width: 160px; height: 160px; }
            .joystick-base { width: 160px; height: 160px; }
            .joystick-stick { width: 65px; height: 65px; left: 47.5px; top: 47.5px; }
        }
    </style>
</head>
<body>
    <div class="header"><h1>Robot Mecanum</h1></div>

    <div class="status-bar">
        <span><span class="status-dot" id="connDot"></span><span id="connText">...</span></span>
        <span>GPS: <span class="status-dot" id="gpsDot"></span><span id="gpsFixStatus">--</span></span>
    </div>

    <div class="sensors-panel">
        <div class="sensor-group">
            <div class="sensor-label">Accel (g)</div>
            <div class="sensor-values">
                <span class="sensor-axis"><span class="axis-label">X</span> <span id="accX">--</span></span>
                <span class="sensor-axis"><span class="axis-label">Y</span> <span id="accY">--</span></span>
                <span class="sensor-axis"><span class="axis-label">Z</span> <span id="accZ">--</span></span>
            </div>
        </div>
        <div class="sensor-group">
            <div class="sensor-label">Angles</div>
            <div class="sensor-values">
                <span class="sensor-axis"><span class="axis-label">Pitch</span> <span id="pitch">--</span></span>
                <span class="sensor-axis"><span class="axis-label">Roll</span> <span id="roll">--</span></span>
            </div>
        </div>
        <div class="sensor-group">
            <div class="sensor-label">Gyro (°/s)</div>
            <div class="sensor-values">
                <span class="sensor-axis"><span class="axis-label">X</span> <span id="gyrX">--</span></span>
                <span class="sensor-axis"><span class="axis-label">Y</span> <span id="gyrY">--</span></span>
                <span class="sensor-axis"><span class="axis-label">Z</span> <span id="gyrZ">--</span></span>
            </div>
        </div>
    </div>

    <div class="sensors-panel gps-panel">
        <div class="sensor-group">
            <div class="sensor-label">Position</div>
            <div class="sensor-values">
                <span class="sensor-axis"><span class="axis-label">Lat</span> <span id="gpsLat">--</span></span>
                <span class="sensor-axis"><span class="axis-label">Lon</span> <span id="gpsLon">--</span></span>
            </div>
        </div>
        <div class="sensor-group">
            <div class="sensor-label">Vitesse</div>
            <div class="sensor-values">
                <span id="gpsSpeed">--</span> <span style="opacity:0.6">km/h</span>
            </div>
        </div>
        <div class="sensor-group">
            <div class="sensor-label">Cap</div>
            <div class="sensor-values">
                <span id="gpsHeading">--</span> <span style="opacity:0.6" id="gpsCardinal"></span>
            </div>
        </div>
    </div>

    <div class="joysticks-row">
        <div class="joystick-wrapper">
            <div class="joystick-label">Deplacement</div>
            <div class="joystick-container" id="moveJoystick">
                <div class="joystick-base"></div>
                <div class="joystick-stick move" id="moveStick"></div>
            </div>
        </div>

        <div class="joystick-wrapper">
            <div class="joystick-label">Rotation</div>
            <div class="joystick-container" id="rotateJoystick">
                <div class="joystick-base"></div>
                <div class="joystick-stick rotate" id="rotateStick"></div>
            </div>
        </div>
    </div>

    <div class="bottom-bar">
        <div class="telemetry">
            <div class="tel-row">X:<span id="telX">0.00</span> Y:<span id="telY">0.00</span> R:<span id="telR">0.00</span></div>
            <div class="tel-row"><span id="motorStatus">FL:0 FR:0 RL:0 RR:0</span></div>
        </div>

        <div class="speed-control">
            <button class="speed-btn" id="speedDown">-</button>
            <span class="speed-value" id="speedValue">50%</span>
            <button class="speed-btn" id="speedUp">+</button>
        </div>

    </div>

    <script>
        let moveX = 0, moveY = 0, rotation = 0, maxSpeed = 50;
        let moveActive = false, rotateActive = false;
        const STICK_RADIUS = 35;
        const CENTER = 70;

        const moveJoystick = document.getElementById('moveJoystick');
        const moveStick = document.getElementById('moveStick');
        const rotateJoystick = document.getElementById('rotateJoystick');
        const rotateStick = document.getElementById('rotateStick');

        function setupJoystick(container, stick, isRotation) {
            let active = false;

            container.addEventListener('pointerdown', (e) => {
                e.preventDefault();
                container.setPointerCapture(e.pointerId);
                active = true;
                stick.classList.add('active');
                updateStick(e, container, stick, isRotation);
            });

            container.addEventListener('pointermove', (e) => {
                if (!active) return;
                e.preventDefault();
                updateStick(e, container, stick, isRotation);
            });

            container.addEventListener('pointerup', () => resetStick(stick, isRotation));
            container.addEventListener('pointercancel', () => resetStick(stick, isRotation));
            container.addEventListener('pointerleave', () => { if (active) resetStick(stick, isRotation); });

            function resetStick(stick, isRotation) {
                active = false;
                stick.classList.remove('active');
                stick.style.left = '42.5px';
                stick.style.top = '42.5px';
                if (isRotation) { rotation = 0; }
                else { moveX = 0; moveY = 0; }
                updateTelemetry();
                sendCommand();
            }
        }

        function updateStick(e, container, stick, isRotation) {
            const rect = container.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            let x = e.clientX - rect.left - centerX;
            let y = e.clientY - rect.top - centerY;

            const dist = Math.sqrt(x*x + y*y);
            const maxDist = rect.width / 2 - 27;
            if (dist > maxDist) {
                x = x / dist * maxDist;
                y = y / dist * maxDist;
            }

            stick.style.left = (centerX + x - 27.5) + 'px';
            stick.style.top = (centerY + y - 27.5) + 'px';

            if (isRotation) {
                rotation = x / maxDist;
            } else {
                moveX = x / maxDist;
                moveY = -y / maxDist;
            }
            updateTelemetry();
            sendCommand();
        }

        setupJoystick(moveJoystick, moveStick, false);
        setupJoystick(rotateJoystick, rotateStick, true);

        document.getElementById('speedDown').addEventListener('click', () => {
            maxSpeed = Math.max(30, maxSpeed - 10);
            document.getElementById('speedValue').textContent = maxSpeed + '%';
        });

        document.getElementById('speedUp').addEventListener('click', () => {
            maxSpeed = Math.min(100, maxSpeed + 10);
            document.getElementById('speedValue').textContent = maxSpeed + '%';
        });

        function updateTelemetry() {
            document.getElementById('telX').textContent = moveX.toFixed(2);
            document.getElementById('telY').textContent = moveY.toFixed(2);
            document.getElementById('telR').textContent = rotation.toFixed(2);
        }

        let sendTimeout = null;
        function sendCommand() {
            if (sendTimeout) clearTimeout(sendTimeout);
            sendTimeout = setTimeout(async () => {
                try {
                    const res = await fetch('/api/move', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({vx: moveX, vy: moveY, omega: rotation, speed: maxSpeed})
                    });
                    if (res.ok) {
                        const data = await res.json();
                        setConnected(true);
                        if (data.motors) {
                            const m = data.motors;
                            document.getElementById('motorStatus').textContent =
                                `FL:${m.FL?.speed||0} FR:${m.FR?.speed||0} RL:${m.RL?.speed||0} RR:${m.RR?.speed||0}`;
                        }
                    }
                } catch(e) { setConnected(false); }
            }, 50);
        }

        function setConnected(c) {
            document.getElementById('connDot').classList.toggle('connected', c);
            document.getElementById('connText').textContent = c ? 'OK' : 'Err';
        }

        function headingToCardinal(deg) {
            const dirs = ['N', 'NE', 'E', 'SE', 'S', 'SO', 'O', 'NO'];
            return dirs[Math.round(deg / 45) % 8];
        }

        setInterval(async () => {
            try {
                const res = await fetch('/api/sensors');
                if (res.ok) {
                    const d = await res.json();
                    // Accelerometre
                    document.getElementById('accX').textContent = (d.acc_x !== undefined) ? d.acc_x.toFixed(2) : '--';
                    document.getElementById('accY').textContent = (d.acc_y !== undefined) ? d.acc_y.toFixed(2) : '--';
                    document.getElementById('accZ').textContent = (d.acc_z !== undefined) ? d.acc_z.toFixed(2) : '--';
                    // Angles (pitch/roll depuis accelerometre)
                    if (d.acc_x !== undefined && d.acc_y !== undefined && d.acc_z !== undefined) {
                        const pitch = Math.atan2(d.acc_y, Math.sqrt(d.acc_x*d.acc_x + d.acc_z*d.acc_z)) * 180 / Math.PI;
                        const roll = Math.atan2(-d.acc_x, d.acc_z) * 180 / Math.PI;
                        document.getElementById('pitch').textContent = pitch.toFixed(1) + '°';
                        document.getElementById('roll').textContent = roll.toFixed(1) + '°';
                    }
                    // Gyroscope
                    document.getElementById('gyrX').textContent = (d.gyr_x !== undefined) ? d.gyr_x.toFixed(1) : '--';
                    document.getElementById('gyrY').textContent = (d.gyr_y !== undefined) ? d.gyr_y.toFixed(1) : '--';
                    document.getElementById('gyrZ').textContent = (d.gyr_z !== undefined) ? d.gyr_z.toFixed(1) : '--';
                    // GPS
                    const hasFix = d.latitude !== undefined && (d.latitude !== 0 || d.longitude !== 0);
                    document.getElementById('gpsDot').classList.toggle('connected', hasFix);
                    document.getElementById('gpsFixStatus').textContent = hasFix ? 'FIX' : 'NO FIX';
                    document.getElementById('gpsLat').textContent = (d.latitude !== undefined) ? d.latitude.toFixed(6) : '--';
                    document.getElementById('gpsLon').textContent = (d.longitude !== undefined) ? d.longitude.toFixed(6) : '--';
                    document.getElementById('gpsSpeed').textContent = (d.speed_kmh !== undefined) ? d.speed_kmh.toFixed(1) : '--';
                    document.getElementById('gpsHeading').textContent = (d.heading !== undefined) ? d.heading.toFixed(0) + '°' : '--';
                    document.getElementById('gpsCardinal').textContent = (d.heading !== undefined) ? headingToCardinal(d.heading) : '';
                }
            } catch(e) {}
        }, 1000);

        sendCommand();
    </script>
</body>
</html>
"""


class WebServer:
    """Serveur web Flask pour contrôle du robot"""

    def __init__(self, motor_controller=None, sensor_receiver=None, host='0.0.0.0', port=8085):
        self.app = Flask(__name__)
        self.motor_controller = motor_controller
        self.sensor_receiver = sensor_receiver
        self.host = host
        self.port = port
        self._thread = None
        self._running = False
        self._register_routes()

    def _register_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)

        @self.app.route('/api/move', methods=['POST'])
        def api_move():
            try:
                data = request.get_json()
                vx = float(data.get('vx', 0))
                vy = float(data.get('vy', 0))
                omega = float(data.get('omega', 0))
                speed = int(data.get('speed', 50))

                motors = None
                if self.motor_controller:
                    self.motor_controller.move(vx, vy, omega, speed)
                    motors = self.motor_controller.get_status()

                return jsonify({'status': 'ok', 'motors': motors})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400

        @self.app.route('/api/sensors')
        def api_sensors():
            if self.sensor_receiver:
                data = self.sensor_receiver.get_last_data()
                if data:
                    return jsonify(asdict(data))
            return jsonify({})

        @self.app.route('/api/status')
        def api_status():
            return jsonify({
                'motors_connected': self.motor_controller is not None,
                'sensors_connected': self.sensor_receiver is not None
            })

        @self.app.route('/api/stop', methods=['POST'])
        def api_stop():
            if self.motor_controller:
                self.motor_controller.stop()
            return jsonify({'status': 'ok'})

    def start(self, threaded=True):
        local_ip = get_local_ip()
        url = f"http://{local_ip}:{self.port}"

        if threaded:
            self._running = True
            self._thread = threading.Thread(
                target=lambda: self.app.run(host=self.host, port=self.port, debug=False, use_reloader=False),
                daemon=True
            )
            self._thread.start()
            print(f"Serveur web: {url}")
        else:
            print(f"Serveur web: {url}")
            self.app.run(host=self.host, port=self.port, debug=False)

        return url

    def get_url(self):
        return f"http://{get_local_ip()}:{self.port}"

    def stop(self):
        self._running = False


if __name__ == "__main__":
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    print("=" * 40)
    print("  Test Serveur Web - Double Joystick")
    print("=" * 40)

    # Essayer de charger le motor_controller pour test
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

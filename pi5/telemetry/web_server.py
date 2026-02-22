"""
Flask web server for Mecanum robot control via smartphone
Dual touch joystick interface + real-time telemetry
+ 360deg lidar view + waypoint management + autonomous patrol
"""

import json
import time
import socket
import threading
from functools import wraps
from flask import Flask, render_template_string, jsonify, request, Response
from werkzeug.security import check_password_hash
from dataclasses import asdict


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


HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no, viewport-fit=cover">
    <title>Robot Mecanum</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; -webkit-user-select: none; user-select: none; }
        .joystick-container { touch-action: none; }
        html { height: 100%; }
        body { font-family: -apple-system, sans-serif; background: #1a1a2e; color: #fff; min-height: 100%; display: flex; flex-direction: column; overflow-y: auto; padding-top: env(safe-area-inset-top); padding-bottom: env(safe-area-inset-bottom); -webkit-overflow-scrolling: touch; }

        .header { text-align: center; padding: 8px; background: rgba(0,0,0,0.3); flex-shrink: 0; }
        .header h1 { font-size: 1.1em; font-weight: 500; }

        .status-bar { display: flex; flex-direction: column; padding: 4px 8px; background: rgba(0,0,0,0.2); font-size: 0.75em; flex-shrink: 0; gap: 3px; }
        .status-row { display: flex; justify-content: space-around; flex-wrap: wrap; gap: 4px; }
        .status-dot { width: 8px; height: 8px; border-radius: 50%; background: #f44; display: inline-block; margin-right: 4px; vertical-align: middle; }
        .status-dot.connected { background: #4f4; }
        .mode-badge { padding: 2px 8px; border-radius: 10px; font-size: 0.85em; font-weight: bold; }
        .mode-badge.manual { background: #357abd; }
        .mode-badge.patrol { background: #4a9; }
        .mode-badge.avoiding { background: #e94; }
        .mode-badge.blocked { background: #e44; }

        .sensors-panel { display: flex; justify-content: space-around; padding: 8px; background: rgba(0,0,0,0.15); font-size: 0.7em; font-family: monospace; flex-shrink: 0; }
        .sensor-group { text-align: center; }
        .sensor-label { opacity: 0.6; margin-bottom: 2px; }
        .sensor-values { display: flex; gap: 8px; }
        .axis-label { opacity: 0.5; }

        .joysticks-row { display: flex; justify-content: space-around; align-items: center; padding: 10px; min-height: 160px; }

        .joystick-wrapper { text-align: center; }
        .joystick-label { font-size: 0.85em; margin-bottom: 10px; opacity: 0.8; }
        .joystick-container { position: relative; width: 140px; height: 140px; touch-action: none; }
        .joystick-base { position: absolute; width: 140px; height: 140px; border-radius: 50%; background: rgba(255,255,255,0.1); border: 2px solid rgba(255,255,255,0.3); }
        .joystick-stick { position: absolute; width: 55px; height: 55px; border-radius: 50%; left: 42.5px; top: 42.5px; pointer-events: none; transition: transform 0.1s; }
        .joystick-stick.move { background: linear-gradient(145deg, #4a90d9, #357abd); box-shadow: 0 3px 12px rgba(74,144,217,0.5); }
        .joystick-stick.rotate { background: linear-gradient(145deg, #e94560, #c73e54); box-shadow: 0 3px 12px rgba(233,69,96,0.5); }
        .joystick-stick.active { transform: scale(1.1); }

        .bottom-bar { display: flex; justify-content: space-between; align-items: center; padding: 12px 15px; background: rgba(0,0,0,0.5); flex-shrink: 0; }

        .telemetry { font-size: 0.7em; font-family: monospace; }
        .tel-row { margin: 2px 0; }

        .speed-control { display: flex; align-items: center; gap: 8px; }
        .speed-btn { width: 38px; height: 38px; border-radius: 50%; border: 2px solid rgba(255,255,255,0.4); background: transparent; color: #fff; font-size: 1.3em; touch-action: manipulation; }
        .speed-btn:active { background: rgba(255,255,255,0.2); }
        .speed-value { font-size: 1.1em; min-width: 50px; text-align: center; font-weight: bold; }

        /* --- Lidar + Patrol section --- */
        .section-title { font-size: 0.85em; padding: 6px 15px 2px; opacity: 0.7; }

        .lidar-section { display: flex; flex-direction: column; align-items: center; padding: 6px; background: rgba(0,0,0,0.1); }
        .lidar-canvas-wrap { position: relative; }
        #lidarCanvas { background: #0a0a1a; border-radius: 8px; border: 1px solid rgba(255,255,255,0.15); }
        .lidar-info { font-size: 0.65em; font-family: monospace; opacity: 0.6; margin-top: 4px; }

        .patrol-section { padding: 8px 15px; background: rgba(0,0,0,0.15); }
        .patrol-controls { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 8px; }
        .patrol-btn { padding: 8px 14px; border-radius: 6px; border: 1px solid rgba(255,255,255,0.3); background: rgba(255,255,255,0.1); color: #fff; font-size: 0.8em; cursor: pointer; touch-action: manipulation; }
        .patrol-btn:active { background: rgba(255,255,255,0.25); }
        .patrol-btn.start { background: rgba(74,153,120,0.4); border-color: #4a9; }
        .patrol-btn.stop { background: rgba(233,69,96,0.4); border-color: #e94560; }
        .patrol-btn.record { background: rgba(74,144,217,0.4); border-color: #4a90d9; }

        .waypoint-list { font-size: 0.7em; font-family: monospace; max-height: 120px; overflow-y: auto; }
        .wp-item { padding: 3px 6px; border-radius: 3px; margin: 2px 0; display: flex; justify-content: space-between; }
        .wp-item.active { background: rgba(74,153,120,0.3); }
        .wp-delete { color: #f66; cursor: pointer; margin-left: 8px; }

        .patrol-status { font-size: 0.7em; font-family: monospace; opacity: 0.8; margin-top: 6px; white-space: pre-line; }

        .map-section { display: flex; flex-direction: column; align-items: center; padding: 6px; background: rgba(0,0,0,0.1); }
        #mapCanvas { background: #0a0a1a; border-radius: 8px; border: 1px solid rgba(255,255,255,0.15); }
        .map-info { font-size: 0.65em; font-family: monospace; opacity: 0.6; margin-top: 4px; }

        .footer-space { height: 20px; flex-shrink: 0; }
        .shutdown-section { display: flex; justify-content: center; padding: 10px; }
        .shutdown-btn { padding: 8px 20px; border-radius: 6px; border: 1px solid rgba(233,69,96,0.6); background: rgba(233,69,96,0.15); color: #e94560; font-size: 0.8em; cursor: pointer; touch-action: manipulation; }
        .shutdown-btn:active { background: rgba(233,69,96,0.35); }

        .toast { position: fixed; top: 60px; left: 50%; transform: translateX(-50%); padding: 10px 20px; border-radius: 8px; font-size: 0.85em; z-index: 999; opacity: 0; transition: opacity 0.3s; pointer-events: none; }
        .toast.show { opacity: 1; }
        .toast.error { background: rgba(233,69,96,0.9); color: #fff; }
        .toast.success { background: rgba(74,153,120,0.9); color: #fff; }

        /* --- Camera AI detections --- */
        .detection-section { padding: 8px 15px; background: rgba(0,0,0,0.15); transition: background 0.5s, border-left 0.5s; }
        .detection-section.active-alert { background: rgba(233,69,96,0.12); border-left: 3px solid #e94560; }
        .camera-status { font-size: 0.7em; font-family: monospace; opacity: 0.8; margin-bottom: 4px; display: flex; align-items: center; gap: 8px; }
        .detection-list { font-size: 0.7em; font-family: monospace; max-height: 140px; overflow-y: auto; }
        .detection-item { padding: 3px 6px; border-radius: 3px; margin: 2px 0; }
        .detection-item.alert { background: rgba(233,69,96,0.25); border-left: 2px solid #e94560; }
        .stream-btn { display: block; width: 100%; margin: 6px 0; padding: 5px 10px; border-radius: 5px; border: 1px solid rgba(255,255,255,0.3); background: rgba(255,255,255,0.1); color: #fff; font-size: 0.75em; cursor: pointer; touch-action: manipulation; text-align: center; }
        .stream-btn:active { background: rgba(255,255,255,0.25); }
        /* Stream modal */
        .stream-modal { display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.92); z-index: 1000; justify-content: center; align-items: center; }
        .stream-modal.open { display: flex; }
        .stream-modal-inner { position: relative; max-width: 100%; max-height: 100%; text-align: center; }
        .stream-modal img { max-width: 100%; max-height: 90vh; object-fit: contain; border-radius: 6px; }
        .stream-close { position: absolute; top: -32px; right: 0; background: none; border: none; color: #fff; font-size: 1.6em; cursor: pointer; line-height: 1; }

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
        <div class="status-row">
            <span><span class="status-dot" id="connDot"></span>WiFi: <span id="connText">...</span></span>
            <span>GPS: <span class="status-dot" id="gpsDot"></span><span id="gpsFixStatus">--</span> | <span id="gpsSats">0</span> sat</span>
            <span class="mode-badge manual" id="modeBadge">MANUEL</span>
        </div>
        <div class="status-row">
            <span>UART: <span class="status-dot" id="uartDot"></span><span id="uartText">--</span></span>
        </div>
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
            <div class="sensor-label">Gyro (deg/s)</div>
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
            <span class="speed-value" id="speedValue">30%</span>
            <button class="speed-btn" id="speedUp">+</button>
        </div>
    </div>

    <!-- Patrol section -->
    <div class="section-title">Patrouille</div>
    <div class="patrol-section">
        <div class="patrol-controls">
            <button class="patrol-btn record" onclick="recordWaypoint()">Enregistrer pos.</button>
            <button class="patrol-btn start" onclick="startPatrol()">Lancer</button>
            <button class="patrol-btn stop" onclick="stopPatrol()">Arrêter</button>
            <button class="patrol-btn" onclick="deleteLastWaypoint()">Suppr. dernier</button>
        </div>
        <div class="waypoint-list" id="waypointList">Aucun waypoint</div>
        <div class="patrol-status" id="patrolStatus">--</div>
    </div>

    <!-- Map section -->
    <div class="section-title">Carte GPS</div>
    <div class="map-section">
        <canvas id="mapCanvas" width="280" height="280"></canvas>
        <div class="map-info"><span id="mapInfo">En attente...</span></div>
    </div>

    <!-- Lidar section -->
    <div class="section-title">LiDAR 360</div>
    <div class="lidar-section">
        <div class="lidar-canvas-wrap">
            <canvas id="lidarCanvas" width="200" height="200"></canvas>
        </div>
        <div class="lidar-info"><span id="lidarInfo">--</span></div>
    </div>

    <!-- Camera AI detections section -->
    <div class="section-title">Caméra IA</div>
    <div class="detection-section" id="detectionSection">
        <div class="camera-status">
            <span class="status-dot" id="cameraDot"></span>
            <span id="cameraStatus">--</span>
        </div>
        <button class="stream-btn" onclick="openStream()">&#128247; Voir le flux</button>
        <div class="detection-list" id="detectionList">Aucune détection</div>
    </div>

    <!-- Stream modal -->
    <div class="stream-modal" id="streamModal" onclick="if(event.target===this)closeStream()">
        <div class="stream-modal-inner">
            <button class="stream-close" onclick="closeStream()">&#x2715;</button>
            <img id="streamImg" src="" alt="flux caméra" style="transform: rotate(180deg)">
        </div>
    </div>

    <div class="shutdown-section">
        <button class="shutdown-btn" id="shutdownBtn">&#9211; Eteindre le robot</button>
    </div>
    <div class="footer-space"></div>
    <div class="toast" id="toast"></div>

    <script>
        function showToast(msg, type='error', duration=2500) {
            const t = document.getElementById('toast');
            t.textContent = msg;
            t.className = 'toast ' + type + ' show';
            setTimeout(() => t.classList.remove('show'), duration);
        }
        /* ===== Joystick control (unchanged) ===== */
        let moveX = 0, moveY = 0, rotation = 0, maxSpeed = 30;
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
                stick.style.left = '42.5px'; stick.style.top = '42.5px';
                if (isRotation) rotation = 0; else { moveX = 0; moveY = 0; }
                updateTelemetry(); sendCommand();
            }
        }
        function updateStick(e, container, stick, isRotation) {
            const rect = container.getBoundingClientRect();
            const cx = rect.width/2, cy = rect.height/2;
            let x = e.clientX - rect.left - cx, y = e.clientY - rect.top - cy;
            const dist = Math.sqrt(x*x+y*y), maxDist = rect.width/2 - 27;
            if (dist > maxDist) { x = x/dist*maxDist; y = y/dist*maxDist; }
            stick.style.left = (cx+x-27.5)+'px'; stick.style.top = (cy+y-27.5)+'px';
            if (isRotation) rotation = x/maxDist;
            else { moveX = x/maxDist; moveY = -y/maxDist; }
            updateTelemetry(); sendCommand();
        }
        setupJoystick(moveJoystick, moveStick, false);
        setupJoystick(rotateJoystick, rotateStick, true);

        document.getElementById('speedDown').addEventListener('click', () => {
            maxSpeed = Math.max(10, maxSpeed - 5);
            document.getElementById('speedValue').textContent = maxSpeed + '%';
        });
        document.getElementById('speedUp').addEventListener('click', () => {
            maxSpeed = Math.min(100, maxSpeed + 5);
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
        let _wasConnected = null;
        function setConnected(c) {
            document.getElementById('connDot').classList.toggle('connected', c);
            document.getElementById('connText').textContent = c ? 'OK' : 'Hors ligne';
            if (_wasConnected !== null && _wasConnected !== c) {
                if (c) {
                    // Reconnected: flush joystick state and send stop immediately
                    moveX = 0; moveY = 0; rotation = 0;
                    const sticks = [moveStick, rotateStick];
                    sticks.forEach(s => { s.style.left = '42.5px'; s.style.top = '42.5px'; });
                    fetch('/api/stop', { method: 'POST' });
                    showToast("Connexion rétablie", 'success', 4000);
                } else {
                    showToast("Connexion perdue - robot arrêté", 'error', 4000);
                }
            }
            _wasConnected = c;
        }

        /* ===== UART status polling ===== */
        setInterval(async () => {
            try {
                const res = await fetch('/api/status');
                if (!res.ok) return;
                const d = await res.json();
                const ok = d.uart_ok === true;
                document.getElementById('uartDot').classList.toggle('connected', ok);
                document.getElementById('uartText').textContent = ok ? 'Pico OK' : 'Pico --';
            } catch(e) {}
        }, 2000);
        function headingToCardinal(deg) {
            const dirs = ['N','NE','E','SE','S','SO','O','NO'];
            return dirs[Math.round(deg/45) % 8];
        }

        /* ===== Sensor polling ===== */
        setInterval(async () => {
            try {
                const res = await fetch('/api/sensors');
                if (!res.ok) return;
                const d = await res.json();
                document.getElementById('accX').textContent = (d.acc_x !== undefined) ? d.acc_x.toFixed(2) : '--';
                document.getElementById('accY').textContent = (d.acc_y !== undefined) ? d.acc_y.toFixed(2) : '--';
                document.getElementById('accZ').textContent = (d.acc_z !== undefined) ? d.acc_z.toFixed(2) : '--';
                if (d.acc_x !== undefined && d.acc_y !== undefined && d.acc_z !== undefined) {
                    const pitch = Math.atan2(d.acc_y, Math.sqrt(d.acc_x*d.acc_x + d.acc_z*d.acc_z)) * 180 / Math.PI;
                    const roll = Math.atan2(-d.acc_x, d.acc_z) * 180 / Math.PI;
                    document.getElementById('pitch').textContent = pitch.toFixed(1) + 'deg';
                    document.getElementById('roll').textContent = roll.toFixed(1) + 'deg';
                }
                document.getElementById('gyrX').textContent = (d.gyr_x !== undefined) ? d.gyr_x.toFixed(1) : '--';
                document.getElementById('gyrY').textContent = (d.gyr_y !== undefined) ? d.gyr_y.toFixed(1) : '--';
                document.getElementById('gyrZ').textContent = (d.gyr_z !== undefined) ? d.gyr_z.toFixed(1) : '--';
                const hasFix = d.has_fix === true || (d.latitude !== undefined && (d.latitude !== 0 || d.longitude !== 0));
                document.getElementById('gpsDot').classList.toggle('connected', hasFix);
                document.getElementById('gpsFixStatus').textContent = hasFix ? 'FIX' : 'NO FIX';
                document.getElementById('gpsSats').textContent = (d.satellites !== undefined) ? d.satellites : '0';
                document.getElementById('gpsLat').textContent = (d.latitude !== undefined) ? d.latitude.toFixed(6) : '--';
                document.getElementById('gpsLon').textContent = (d.longitude !== undefined) ? d.longitude.toFixed(6) : '--';
                const speedKmh = (d.speed_kmh !== undefined && d.speed_kmh > 0.5) ? d.speed_kmh : 0;
                document.getElementById('gpsSpeed').textContent = speedKmh.toFixed(1);
                document.getElementById('gpsHeading').textContent = (d.heading !== undefined) ? d.heading.toFixed(0) + 'deg' : '--';
                document.getElementById('gpsCardinal').textContent = (d.heading !== undefined) ? headingToCardinal(d.heading) : '';
                if (hasFix) {
                    _mapRawPos = { lat: d.latitude, lon: d.longitude };
                    _rawPosBuffer.push({ lat: d.latitude, lon: d.longitude });
                    if (_rawPosBuffer.length > 3) _rawPosBuffer.shift();
                    const avgLat = _rawPosBuffer.reduce((s,p) => s + p.lat, 0) / _rawPosBuffer.length;
                    const avgLon = _rawPosBuffer.reduce((s,p) => s + p.lon, 0) / _rawPosBuffer.length;
                    _mapCurrentPos = { lat: avgLat, lon: avgLon, heading: d.heading || 0 };
                } else {
                    _mapRawPos = null; _mapCurrentPos = null; _rawPosBuffer = [];
                }
                drawMap();
            } catch(e) {}
        }, 1000);

        /* ===== Lidar canvas ===== */
        const lidarCanvas = document.getElementById('lidarCanvas');
        const ctx = lidarCanvas.getContext('2d');
        const CW = lidarCanvas.width, CH = lidarCanvas.height;
        const CENTER_X = CW/2, CENTER_Y = CH/2;
        const MAX_RANGE = 6000; // mm
        const SCALE = (CW/2 - 10) / MAX_RANGE;

        function drawLidar(points) {
            ctx.clearRect(0, 0, CW, CH);
            ctx.fillStyle = '#0a0a1a';
            ctx.fillRect(0, 0, CW, CH);

            // Reference circles
            ctx.strokeStyle = 'rgba(255,255,255,0.08)';
            ctx.lineWidth = 0.5;
            [1000, 2000, 3000, 4000, 5000, 6000].forEach(r => {
                ctx.beginPath();
                ctx.arc(CENTER_X, CENTER_Y, r * SCALE, 0, Math.PI*2);
                ctx.stroke();
            });

            // Cross
            ctx.strokeStyle = 'rgba(255,255,255,0.1)';
            ctx.beginPath(); ctx.moveTo(CENTER_X, 0); ctx.lineTo(CENTER_X, CH); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(0, CENTER_Y); ctx.lineTo(CW, CENTER_Y); ctx.stroke();

            // Robot marker
            ctx.fillStyle = '#4f4';
            ctx.beginPath(); ctx.arc(CENTER_X, CENTER_Y, 3, 0, Math.PI*2); ctx.fill();

            // Points
            if (!points || points.length === 0) return;
            for (const p of points) {
                const a_rad = p[0] * Math.PI / 180;
                const d = p[1];
                if (d <= 0) continue;
                const x = CENTER_X + d * Math.sin(a_rad) * SCALE;
                const y = CENTER_Y - d * Math.cos(a_rad) * SCALE;
                // Color by distance
                const t = Math.min(d / MAX_RANGE, 1);
                const r = Math.round(255 * t);
                const g = Math.round(255 * (1-t));
                ctx.fillStyle = `rgb(${r},${g},80)`;
                ctx.fillRect(x-1, y-1, 2, 2);
            }
        }

        setInterval(async () => {
            try {
                const res = await fetch('/api/lidar');
                if (!res.ok) return;
                const data = await res.json();
                drawLidar(data.points || []);
                document.getElementById('lidarInfo').textContent =
                    `${data.num_points || 0} pts | nearest: ${data.nearest_mm ? (data.nearest_mm/1000).toFixed(1)+'m' : '--'}`;
            } catch(e) {
                drawLidar([]);
            }
        }, 200);

        drawLidar([]); // Initial empty

        /* ===== GPS Map ===== */
        const mapCanvas = document.getElementById('mapCanvas');
        const mapCtx = mapCanvas.getContext('2d');
        let _mapWaypoints = [];
        let _mapCurrentPos = null;  // averaged position (3-sample): white triangle
        let _mapRawPos = null;      // raw GPS reading: orange dot
        let _rawPosBuffer = [];     // 3-sample rolling average (client-side)

        function drawMap() {
            const W = mapCanvas.width, H = mapCanvas.height;
            mapCtx.clearRect(0, 0, W, H);
            mapCtx.fillStyle = '#0a0a1a';
            mapCtx.fillRect(0, 0, W, H);

            const all = [
                ..._mapWaypoints.map(w => ({lat: w.latitude, lon: w.longitude})),
                ...(_mapCurrentPos ? [{lat: _mapCurrentPos.lat, lon: _mapCurrentPos.lon}] : []),
                ...(_mapRawPos ? [{lat: _mapRawPos.lat, lon: _mapRawPos.lon}] : [])
            ];

            if (all.length === 0) {
                mapCtx.fillStyle = 'rgba(255,255,255,0.3)';
                mapCtx.font = '12px sans-serif';
                mapCtx.textAlign = 'center';
                mapCtx.fillText('Aucun point GPS', W/2, H/2);
                document.getElementById('mapInfo').textContent = 'En attente...';
                return;
            }

            const lats = all.map(p => p.lat);
            const lons = all.map(p => p.lon);
            const minLat = Math.min(...lats), maxLat = Math.max(...lats);
            const minLon = Math.min(...lons), maxLon = Math.max(...lons);
            const origin = { lat: (minLat + maxLat) / 2, lon: (minLon + maxLon) / 2 };
            const cosLat = Math.cos(origin.lat * Math.PI / 180);
            const dxM = (maxLon - minLon) * cosLat * 111320;
            const dyM = (maxLat - minLat) * 111320;
            const extent = Math.max(dxM, dyM, 10);  // at least 10m visible radius
            const padding = 36;
            const scale = (Math.min(W, H) / 2 - padding) / extent;

            function toCanvas(lat, lon) {
                const x = (lon - origin.lon) * cosLat * 111320;
                const y = -(lat - origin.lat) * 111320;
                return { cx: W/2 + x * scale, cy: H/2 + y * scale };
            }

            // Scale bar (bottom-left)
            const barM = [1, 2, 5, 10, 20, 50].find(v => v * scale >= 30) || 50;
            const barPx = barM * scale;
            mapCtx.strokeStyle = 'rgba(255,255,255,0.5)';
            mapCtx.lineWidth = 2;
            mapCtx.beginPath();
            mapCtx.moveTo(10, H - 10); mapCtx.lineTo(10 + barPx, H - 10);
            mapCtx.stroke();
            mapCtx.fillStyle = 'rgba(255,255,255,0.5)';
            mapCtx.font = '9px monospace';
            mapCtx.textAlign = 'left';
            mapCtx.fillText(barM + 'm', 14, H - 14);

            // North indicator (top-right)
            mapCtx.fillStyle = 'rgba(255,255,255,0.6)';
            mapCtx.font = '11px sans-serif';
            mapCtx.textAlign = 'right';
            mapCtx.fillText('N\u2191', W - 6, 16);

            // Legend (top-left) — semi-transparent background for readability
            mapCtx.fillStyle = 'rgba(0,0,0,0.45)';
            mapCtx.fillRect(2, 2, 68, 78);
            mapCtx.font = '9px monospace';
            mapCtx.textAlign = 'left';
            mapCtx.fillStyle = '#f93';
            mapCtx.fillText('\u25cf brut',  6, 14);
            mapCtx.fillStyle = 'rgba(255,255,255,0.9)';
            mapCtx.fillText('\u25b2 moy.',  6, 26);
            mapCtx.fillStyle = '#4a90d9';
            mapCtx.fillText('\u25cf WP',    6, 38);
            mapCtx.fillStyle = '#4f4';
            mapCtx.fillText('\u25cf cible', 6, 50);
            mapCtx.fillStyle = 'rgba(255,255,255,0.7)';
            mapCtx.fillText('\u2014 leg',      6, 62);
            mapCtx.fillStyle = 'rgba(255,255,255,0.25)';
            mapCtx.fillText('- - route',   6, 74);

            // Full patrol route: faint dashed lines between all waypoints
            if (_mapWaypoints.length > 1) {
                mapCtx.strokeStyle = 'rgba(255,255,255,0.15)';
                mapCtx.lineWidth = 1;
                mapCtx.setLineDash([3, 5]);
                mapCtx.beginPath();
                _mapWaypoints.forEach((wp, i) => {
                    const {cx, cy} = toCanvas(wp.latitude, wp.longitude);
                    if (i === 0) mapCtx.moveTo(cx, cy); else mapCtx.lineTo(cx, cy);
                });
                mapCtx.stroke();
                mapCtx.setLineDash([]);
            }

            // Current navigation leg: averaged position → active waypoint (solid, bright)
            if (_mapCurrentPos) {
                const activeWP = _mapWaypoints.find(w => w.active);
                if (activeWP) {
                    const {cx: x1, cy: y1} = toCanvas(_mapCurrentPos.lat, _mapCurrentPos.lon);
                    const {cx: x2, cy: y2} = toCanvas(activeWP.latitude, activeWP.longitude);
                    mapCtx.strokeStyle = 'rgba(255,255,255,0.7)';
                    mapCtx.lineWidth = 1.5;
                    mapCtx.beginPath();
                    mapCtx.moveTo(x1, y1);
                    mapCtx.lineTo(x2, y2);
                    mapCtx.stroke();
                }
            }

            // Waypoints
            _mapWaypoints.forEach(wp => {
                const {cx, cy} = toCanvas(wp.latitude, wp.longitude);
                mapCtx.beginPath();
                mapCtx.arc(cx, cy, wp.active ? 8 : 6, 0, Math.PI*2);
                mapCtx.fillStyle = wp.active ? '#4f4' : '#4a90d9';
                mapCtx.fill();
                mapCtx.strokeStyle = '#fff';
                mapCtx.lineWidth = 1;
                mapCtx.stroke();
                mapCtx.fillStyle = '#fff';
                mapCtx.font = '9px monospace';
                mapCtx.textAlign = 'center';
                mapCtx.fillText(wp.label, cx, cy - 11);
            });

            // Raw GPS position: orange circle (shows jitter amplitude)
            if (_mapRawPos) {
                const {cx, cy} = toCanvas(_mapRawPos.lat, _mapRawPos.lon);
                mapCtx.beginPath();
                mapCtx.arc(cx, cy, 4, 0, Math.PI*2);
                mapCtx.fillStyle = '#f93';
                mapCtx.fill();
                mapCtx.strokeStyle = 'rgba(255,255,255,0.4)';
                mapCtx.lineWidth = 1;
                mapCtx.stroke();
            }

            // Averaged GPS position: white triangle (what the pilot actually uses)
            if (_mapCurrentPos) {
                const {cx, cy} = toCanvas(_mapCurrentPos.lat, _mapCurrentPos.lon);
                const headRad = (_mapCurrentPos.heading || 0) * Math.PI / 180;
                mapCtx.save();
                mapCtx.translate(cx, cy);
                mapCtx.rotate(headRad);
                mapCtx.beginPath();
                mapCtx.moveTo(0, -9);
                mapCtx.lineTo(6, 7);
                mapCtx.lineTo(0, 3);
                mapCtx.lineTo(-6, 7);
                mapCtx.closePath();
                mapCtx.fillStyle = '#fff';
                mapCtx.fill();
                mapCtx.restore();
            }

            const nWP = _mapWaypoints.length;
            let posStr = 'GPS: no fix';
            if (_mapCurrentPos) {
                // Jitter: distance between raw and averaged position
                let jitterStr = '';
                if (_mapRawPos) {
                    const cosL = Math.cos(_mapCurrentPos.lat * Math.PI / 180);
                    const dLat = (_mapRawPos.lat - _mapCurrentPos.lat) * 111320;
                    const dLon = (_mapRawPos.lon - _mapCurrentPos.lon) * cosL * 111320;
                    jitterStr = ` \u0394${Math.sqrt(dLat*dLat + dLon*dLon).toFixed(1)}m`;
                }
                // Distance and bearing from averaged position to active waypoint
                let distStr = '';
                const activeWP = _mapWaypoints.find(w => w.active);
                if (activeWP) {
                    const R = 6371000;
                    const lat1r = _mapCurrentPos.lat * Math.PI/180;
                    const lat2r = activeWP.latitude  * Math.PI/180;
                    const dLatR = (activeWP.latitude  - _mapCurrentPos.lat) * Math.PI/180;
                    const dLonR = (activeWP.longitude - _mapCurrentPos.lon) * Math.PI/180;
                    const a = Math.sin(dLatR/2)**2 + Math.cos(lat1r)*Math.cos(lat2r)*Math.sin(dLonR/2)**2;
                    const distM = R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
                    // Bearing to active WP
                    const y = Math.sin(dLonR) * Math.cos(lat2r);
                    const x = Math.cos(lat1r)*Math.sin(lat2r) - Math.sin(lat1r)*Math.cos(lat2r)*Math.cos(dLonR);
                    const brg = ((Math.atan2(y, x) * 180/Math.PI) + 360) % 360;
                    distStr = ` | cap ${brg.toFixed(0)}\u00b0 dist ${distM.toFixed(1)}m`;
                }
                posStr = `${_mapCurrentPos.lat.toFixed(5)}, ${_mapCurrentPos.lon.toFixed(5)}${jitterStr}${distStr}`;
            }
            document.getElementById('mapInfo').textContent = `${nWP} WP | ${posStr}`;
        }

        drawMap(); // Initial empty draw

        /* ===== Patrol controls ===== */
        async function recordWaypoint() {
            const t = document.getElementById('toast');
            // Progressive countdown while waiting for server (10s)
            const steps = [
                [0,    'Début enregistrement GPS... (10s)'],
                [3500, '6 secondes restantes...'],
                [6500, '3 secondes restantes...'],
                [8500, 'Finalisation...'],
            ];
            let timers = [];
            steps.forEach(([delay, msg]) => {
                timers.push(setTimeout(() => {
                    t.textContent = msg;
                    t.className = 'toast success show';
                    // Keep each step visible until the next one
                }, delay));
            });
            try {
                const res = await fetch('/api/waypoints/record', {method: 'POST'});
                timers.forEach(id => clearTimeout(id));
                const data = await res.json();
                if (res.ok) {
                    refreshWaypoints();
                    showToast(`Position enregistrée (${data.samples} mesures)`, 'success', 5000);
                } else {
                    showToast(data.message || 'Erreur', 'error');
                }
            } catch(e) {
                timers.forEach(id => clearTimeout(id));
                showToast('Connexion perdue', 'error');
            }
        }
        async function deleteLastWaypoint() {
            try {
                const res = await fetch('/api/waypoints/last', {method: 'DELETE'});
                if (res.ok) refreshWaypoints();
            } catch(e) {}
        }
        async function startPatrol() {
            try { await fetch('/api/patrol/start', {method: 'POST'}); } catch(e) {}
        }
        async function stopPatrol() {
            try { await fetch('/api/patrol/stop', {method: 'POST'}); } catch(e) {}
        }

        async function refreshWaypoints() {
            try {
                const res = await fetch('/api/waypoints');
                if (!res.ok) return;
                const data = await res.json();
                _mapWaypoints = data.waypoints || [];
                drawMap();
                const list = document.getElementById('waypointList');
                list.textContent = '';
                if (!data.waypoints || data.waypoints.length === 0) {
                    list.textContent = 'Aucun waypoint';
                    return;
                }
                data.waypoints.forEach(wp => {
                    const div = document.createElement('div');
                    div.className = 'wp-item' + (wp.active ? ' active' : '');
                    const label = document.createElement('span');
                    label.textContent = wp.label + ': ' + wp.latitude.toFixed(5) + ', ' + wp.longitude.toFixed(5);
                    const del = document.createElement('span');
                    del.className = 'wp-delete';
                    del.textContent = 'X';
                    del.addEventListener('click', () => deleteWaypoint(wp.index));
                    div.appendChild(label);
                    div.appendChild(del);
                    list.appendChild(div);
                });
            } catch(e) {}
        }
        async function deleteWaypoint(idx) {
            try {
                await fetch('/api/waypoints/' + idx, {method: 'DELETE'});
                refreshWaypoints();
            } catch(e) {}
        }

        // Patrol status polling
        setInterval(async () => {
            try {
                const res = await fetch('/api/patrol/status');
                if (!res.ok) return;
                const s = await res.json();
                const badge = document.getElementById('modeBadge');
                const statusEl = document.getElementById('patrolStatus');

                // Debug nav line: always built when dbg values present
                const hasDbg = s.dbg_bearing !== undefined;
                const dbgLine = hasDbg
                    ? `hdg ${s.dbg_avg_heading}\u00b0 \u2192 cap ${s.dbg_bearing}\u00b0 err ${s.dbg_heading_error}\u00b0 \u03c9${s.dbg_omega} v${s.dbg_speed}m/s`
                    : '';

                if (s.state === 'IDLE') {
                    badge.textContent = 'MANUEL';
                    badge.className = 'mode-badge manual';
                    statusEl.textContent = 'Inactif';
                } else if (s.state === 'NAVIGATING') {
                    badge.textContent = 'PATROUILLE';
                    badge.className = 'mode-badge patrol';
                    const line1 = `WP ${s.waypoint_index+1}/${s.waypoint_count} | dist: ${s.distance_m ? s.distance_m+'m' : '--'}`;
                    statusEl.textContent = dbgLine ? `${line1}\n${dbgLine}` : line1;
                } else if (s.state === 'AVOIDING') {
                    badge.textContent = 'EVITEMENT';
                    badge.className = 'mode-badge avoiding';
                    statusEl.textContent = `Évitement obstacle - WP ${s.waypoint_index+1}/${s.waypoint_count}`;
                } else if (s.state === 'PATROL_COMPLETE') {
                    badge.textContent = 'TERMINE';
                    badge.className = 'mode-badge manual';
                    statusEl.textContent = 'Patrouille terminée';
                } else {
                    badge.textContent = s.state;
                    badge.className = 'mode-badge manual';
                }
            } catch(e) {}
        }, 1000);

        // Shutdown button
        document.getElementById('shutdownBtn').addEventListener('click', () => {
            if (!confirm('Eteindre le robot proprement ?')) return;
            fetch('/api/shutdown', { method: 'POST' })
                .then(() => showToast("Extinction en cours... Vous pouvez couper l'alimentation dans 10s.", 'success', 8000))
                .catch(() => showToast("Erreur lors de l'extinction", 'error'));
        });

        /* ===== Camera AI detections polling ===== */
        const _alertLabels = new Set(['person','cat','dog','horse','sheep','cow','bear','zebra','giraffe']);
        let _lastDetectionTs = 0;
        setInterval(async () => {
            try {
                const res = await fetch('/api/detections');
                if (!res.ok) return;
                const data = await res.json();

                document.getElementById('cameraDot').classList.toggle('connected', data.camera_ok);

                const dets = data.detections || [];

                // Toast for new alerts since last poll
                const newAlerts = dets.filter(d => d.is_alert && d.timestamp > _lastDetectionTs);
                if (newAlerts.length > 0) {
                    const labels = [...new Set(newAlerts.map(d => d.label))];
                    showToast('Détection : ' + labels.join(', '), 'error', 4000);
                }
                if (dets.length > 0) {
                    _lastDetectionTs = Math.max(...dets.map(d => d.timestamp));
                }

                // Persistent red background while a person/animal was seen in the last 15s
                const hasRecentAlert = dets.some(d => _alertLabels.has(d.label) && d.age_s <= 15);
                document.getElementById('detectionSection').classList.toggle('active-alert', hasRecentAlert);

                const list = document.getElementById('detectionList');
                if (dets.length === 0) {
                    list.textContent = 'Aucune détection (60s)';
                    document.getElementById('cameraStatus').textContent =
                        data.camera_ok ? 'En ligne' : 'Erreur caméra';
                    return;
                }
                list.innerHTML = '';
                dets.slice(0, 10).forEach(d => {
                    const div = document.createElement('div');
                    div.className = 'detection-item' + (d.is_alert ? ' alert' : '');
                    const age = d.age_s < 60 ? d.age_s + 's' : Math.round(d.age_s / 60) + 'min';
                    div.textContent = d.label + ' ' + Math.round(d.confidence * 100) + '% — ' + age;
                    list.appendChild(div);
                });
                document.getElementById('cameraStatus').textContent = dets.length + ' détection(s) (60s)';
            } catch(e) {}
        }, 2000);

        /* ===== Stream modal ===== */
        function openStream() {
            document.getElementById('streamImg').src = '/api/stream';
            document.getElementById('streamModal').classList.add('open');
        }
        function closeStream() {
            document.getElementById('streamImg').src = '';  // stops the MJPEG connection
            document.getElementById('streamModal').classList.remove('open');
        }

        // Heartbeat: send current joystick state every 200ms to feed the watchdog
        setInterval(sendCommand, 200);

        // Initial load
        refreshWaypoints();
        sendCommand();
    </script>
</body>
</html>
"""


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
        self._ssl_context = (ssl_cert, ssl_key) if ssl_cert and ssl_key else None
        self._register_routes()

    def _require_auth(self, f):
        """Decorator: HTTP Basic Auth if username/hash are configured"""
        @wraps(f)
        def decorated(*args, **kwargs):
            if not self._auth_username or not self._auth_password_hash:
                return f(*args, **kwargs)
            auth = request.authorization
            if not auth or auth.username != self._auth_username \
               or not check_password_hash(self._auth_password_hash, auth.password):
                return Response(
                    'Authentification requise', 401,
                    {'WWW-Authenticate': 'Basic realm="Robot Mecanum"'}
                )
            return f(*args, **kwargs)
        return decorated

    def _register_routes(self):

        # Apply auth to all requests (before_request hook)
        @self.app.before_request
        def check_auth():
            if not self._auth_username or not self._auth_password_hash:
                return None
            auth = request.authorization
            if not auth or auth.username != self._auth_username \
               or not check_password_hash(self._auth_password_hash, auth.password):
                return Response(
                    'Authentification requise', 401,
                    {'WWW-Authenticate': 'Basic realm="Robot Mecanum"'}
                )

        # --- Existing routes ---

        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)

        @self.app.route('/api/move', methods=['POST'])
        def api_move():
            try:
                data = request.get_json()
                vx = max(-1.0, min(1.0, float(data.get('vx', 0))))
                vy = max(-1.0, min(1.0, float(data.get('vy', 0))))
                omega = max(-1.0, min(1.0, float(data.get('omega', 0))))
                speed = max(0, min(100, int(data.get('speed', 50))))
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
                return jsonify({'detections': [], 'camera_ok': False})
            return jsonify({
                'detections': self.camera.get_recent_detections(max_age_s=60.0),
                'camera_ok': not self.camera.has_error(),
            })

        @self.app.route('/api/stream')
        def api_stream():
            if not self.camera or self.camera.has_error():
                return Response('Camera non disponible', status=503)
            return Response(
                self.camera.generate_stream(),
                mimetype='multipart/x-mixed-replace; boundary=frame',
            )

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

            # Average N GPS readings to reduce position error
            import time
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
            except Exception:
                pass
            return jsonify({'status': 'ok', 'index': idx, 'samples': len(lats)})

        @self.app.route('/api/waypoints/<int:idx>', methods=['DELETE'])
        def api_delete_waypoint(idx):
            if not self.patrol_manager:
                return jsonify({'status': 'error'}), 400
            self.patrol_manager.delete_waypoint(idx)
            try:
                self.patrol_manager.save(self._patrol_file)
            except Exception:
                pass
            return jsonify({'status': 'ok'})

        @self.app.route('/api/waypoints/last', methods=['DELETE'])
        def api_delete_last_waypoint():
            if not self.patrol_manager or self.patrol_manager.count == 0:
                return jsonify({'status': 'error'}), 400
            self.patrol_manager.delete_waypoint(self.patrol_manager.count - 1)
            try:
                self.patrol_manager.save(self._patrol_file)
            except Exception:
                pass
            return jsonify({'status': 'ok'})

        # --- System routes ---

        @self.app.route('/api/shutdown', methods=['POST'])
        def api_shutdown():
            import subprocess
            import threading
            def _do_shutdown():
                import time
                time.sleep(1)
                subprocess.run(['sudo', 'shutdown', 'now'])
            threading.Thread(target=_do_shutdown, daemon=True).start()
            return jsonify({'status': 'ok', 'message': 'Extinction en cours...'})

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
        while self._running:
            time.sleep(0.1)
            if not self.motor_controller:
                continue
            pilot_is_active = (
                self.pilot and
                self.pilot.get_status().get('state') not in ('IDLE',)
            )
            if pilot_is_active:
                continue
            if time.time() - self._last_move_time > self._watchdog_interval:
                self.motor_controller.stop()

    def start(self, threaded=True):
        local_ip = get_local_ip()
        scheme = "https" if self._ssl_context else "http"
        url = f"{scheme}://{local_ip}:{self.port}"
        run_kwargs = dict(host=self.host, port=self.port, debug=False, use_reloader=False)
        if self._ssl_context:
            run_kwargs['ssl_context'] = self._ssl_context

        if threaded:
            self._running = True
            self._thread = threading.Thread(
                target=lambda: self.app.run(**run_kwargs),
                daemon=True
            )
            self._thread.start()
            threading.Thread(target=self._watchdog_loop, daemon=True).start()
            print(f"Serveur web: {url}")
        else:
            print(f"Serveur web: {url}")
            self.app.run(**run_kwargs)

        return url

    def get_url(self):
        scheme = "https" if self._ssl_context else "http"
        return f"{scheme}://{get_local_ip()}:{self.port}"

    def stop(self):
        self._running = False


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

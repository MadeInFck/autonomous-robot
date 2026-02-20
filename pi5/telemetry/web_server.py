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

        .status-bar { display: flex; justify-content: space-around; padding: 6px; background: rgba(0,0,0,0.2); font-size: 0.75em; flex-shrink: 0; flex-wrap: wrap; gap: 4px; }
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

        .patrol-status { font-size: 0.7em; font-family: monospace; opacity: 0.8; margin-top: 6px; }

        .footer-space { height: 20px; flex-shrink: 0; }
        .shutdown-section { display: flex; justify-content: center; padding: 10px; }
        .shutdown-btn { padding: 8px 20px; border-radius: 6px; border: 1px solid rgba(233,69,96,0.6); background: rgba(233,69,96,0.15); color: #e94560; font-size: 0.8em; cursor: pointer; touch-action: manipulation; }
        .shutdown-btn:active { background: rgba(233,69,96,0.35); }

        .toast { position: fixed; top: 60px; left: 50%; transform: translateX(-50%); padding: 10px 20px; border-radius: 8px; font-size: 0.85em; z-index: 999; opacity: 0; transition: opacity 0.3s; pointer-events: none; }
        .toast.show { opacity: 1; }
        .toast.error { background: rgba(233,69,96,0.9); color: #fff; }
        .toast.success { background: rgba(74,153,120,0.9); color: #fff; }

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
        <span>GPS: <span class="status-dot" id="gpsDot"></span><span id="gpsFixStatus">--</span> | <span id="gpsSats">0</span> sat</span>
        <span class="mode-badge manual" id="modeBadge">MANUEL</span>
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

    <!-- Lidar section -->
    <div class="section-title">LiDAR 360</div>
    <div class="lidar-section">
        <div class="lidar-canvas-wrap">
            <canvas id="lidarCanvas" width="200" height="200"></canvas>
        </div>
        <div class="lidar-info"><span id="lidarInfo">--</span></div>
    </div>

    <!-- Patrol section -->
    <div class="section-title">Patrouille</div>
    <div class="patrol-section">
        <div class="patrol-controls">
            <button class="patrol-btn record" onclick="recordWaypoint()">Enregistrer pos.</button>
            <button class="patrol-btn start" onclick="startPatrol()">Lancer</button>
            <button class="patrol-btn stop" onclick="stopPatrol()">Arreter</button>
            <button class="patrol-btn" onclick="deleteLastWaypoint()">Suppr. dernier</button>
        </div>
        <div class="waypoint-list" id="waypointList">Aucun waypoint</div>
        <div class="patrol-status" id="patrolStatus">--</div>
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
        function setConnected(c) {
            document.getElementById('connDot').classList.toggle('connected', c);
            document.getElementById('connText').textContent = c ? 'OK' : 'Err';
        }
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

        /* ===== Patrol controls ===== */
        async function recordWaypoint() {
            try {
                const res = await fetch('/api/waypoints/record', {method: 'POST'});
                const data = await res.json();
                if (res.ok) { refreshWaypoints(); showToast('Position enregistree', 'success'); }
                else showToast(data.message || 'Erreur', 'error');
            } catch(e) { showToast('Connexion perdue', 'error'); }
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

                if (s.state === 'IDLE') {
                    badge.textContent = 'MANUEL';
                    badge.className = 'mode-badge manual';
                    statusEl.textContent = 'Inactif';
                } else if (s.state === 'NAVIGATING') {
                    badge.textContent = 'PATROUILLE';
                    badge.className = 'mode-badge patrol';
                    statusEl.textContent = `WP ${s.waypoint_index+1}/${s.waypoint_count} | dist: ${s.distance_m ? s.distance_m+'m' : '--'} | cap: ${s.bearing ? s.bearing+'deg' : '--'}`;
                } else if (s.state === 'AVOIDING') {
                    badge.textContent = 'EVITEMENT';
                    badge.className = 'mode-badge avoiding';
                    statusEl.textContent = `Evitement obstacle - WP ${s.waypoint_index+1}/${s.waypoint_count}`;
                } else if (s.state === 'PATROL_COMPLETE') {
                    badge.textContent = 'TERMINE';
                    badge.className = 'mode-badge manual';
                    statusEl.textContent = 'Patrouille terminee';
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
        self.host = host
        self.port = port
        self._patrol_file = patrol_file
        self._thread = None
        self._running = False
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

                motors = None
                if self.motor_controller:
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
            return jsonify({
                'motors_connected': self.motor_controller is not None,
                'sensors_connected': self.sensor_receiver is not None,
                'lidar_connected': self.lidar_scanner is not None,
                'patrol_available': self.patrol_manager is not None,
            })

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
            GPS_INTERVAL = 0.1  # 10 Hz
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
            return jsonify({'status': 'ok'})

        @self.app.route('/api/patrol/status')
        def api_patrol_status():
            if not self.pilot:
                return jsonify({'state': 'IDLE', 'waypoint_index': 0, 'waypoint_count': 0})
            return jsonify(self.pilot.get_status())

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

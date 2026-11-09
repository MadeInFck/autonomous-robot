"""
BNO085 IMU driver for Raspberry Pi Pico (MicroPython)
Provides yaw/pitch/roll via on-chip sensor fusion (Rotation Vector).

Communication: I2C using SHTP (Sensor Hub Transport Protocol)
Default I2C address: 0x4A (PS0=low, PS1=low on Adafruit breakout)

The Rotation Vector report fuses accelerometer + gyroscope + magnetometer
and outputs an absolute heading referenced to magnetic north.
"""

import time
import math
import struct

# SHTP channel numbers
_CHAN_COMMAND  = 0
_CHAN_CONTROL  = 2  # set-feature and command requests go here
_CHAN_REPORTS  = 3  # input sensor reports arrive here

# Sensor report IDs
_REPORT_ROTATION_VECTOR = 0x05  # absolute heading (accel+gyro+mag fusion, requires mag calibration)

# Protocol command
_CMD_SET_FEATURE = 0xFD

# Report watchdog. A BNO085 that resets (brown-out, loose contact) comes back
# with every report disabled and stays silent forever unless re-initialised.
_STALE_MS  = 1000  # no report this long: orientation no longer valid
_REINIT_MS = 5000  # no report this long: run the init sequence again


class BNO085:
    """BNO085 9-DOF IMU driver — yaw/pitch/roll from onboard fusion.

    Usage::

        imu = BNO085(i2c)
        while True:
            imu.update()
            print(imu.yaw, imu.heading_accuracy_deg)
    """

    def __init__(self, i2c, addr: int = 0x4A, rate_hz: int = 50):
        """
        Args:
            i2c:     MicroPython I2C instance
            addr:    I2C address (0x4A default, 0x4B if ADR pulled high)
            rate_hz: Rotation vector report rate (10–100 Hz)
        """
        self.i2c   = i2c
        self.addr  = addr
        self._seq  = bytearray(8)   # per-channel SHTP sequence counters
        self._error = False

        # Fusion outputs (updated by update())
        self.yaw   = 0.0   # degrees, 0-360, magnetic north reference
        self.pitch = 0.0   # degrees
        self.roll  = 0.0   # degrees
        self.heading_accuracy_deg = 180.0  # lower = better calibrated
        self.valid = False

        self._rate_hz = rate_hz
        self._init(rate_hz)
        self._last_report_ms = time.ticks_ms()

    # ── SHTP low-level ────────────────────────────────────────────

    def _write_packet(self, channel: int, payload: bytes):
        length = len(payload) + 4
        header = struct.pack('<HBB', length, channel, self._seq[channel])
        self._seq[channel] = (self._seq[channel] + 1) & 0xFF
        try:
            self.i2c.writeto(self.addr, header + payload)
        except OSError as e:
            print(f"BNO085 write error: {e}")
            self._error = True

    def _read_packet(self):
        """Read one SHTP packet using a two-phase I2C read.

        Phase 1 — 4-byte header: determines actual packet length.
        Phase 2 — full packet: only issued when payload is available.
        (Single read of N bytes caused RP2040 to clock-stretch garbage into
        the device when N > actual packet size, corrupting subsequent reads.)

        Returns:
            (channel, payload_bytes) or (None, None) on error / no data.
        """
        # Phase 1: header only
        try:
            header = self.i2c.readfrom(self.addr, 4)
        except OSError:
            return None, None
        length = struct.unpack_from('<H', header)[0] & 0x7FFF
        if length <= 4:
            return None, None  # no payload (length==4) or no data (length==0)
        channel = header[2]
        # Phase 2: full packet
        try:
            buf = self.i2c.readfrom(self.addr, length)
        except OSError:
            return None, None
        payload = bytes(buf[4:]) if len(buf) > 4 else b''
        return channel, payload

    # ── Sensor setup ──────────────────────────────────────────────

    def _enable_report(self, report_id: int, rate_hz: int):
        """Send a Set Feature Command to activate a sensor report."""
        rate_us = 1_000_000 // rate_hz
        # Format: cmd(1) + report_id(1) + flags(1) + change_sens(2)
        #         + report_interval_us(4) + batch_interval(4) + sensor_cfg(4) = 17 bytes
        payload = struct.pack('<BBBHIII',
                              _CMD_SET_FEATURE,
                              report_id,
                              0,         # feature flags
                              0,         # change sensitivity (disabled)
                              rate_us,   # report interval µs
                              0,         # batch interval
                              0)         # sensor-specific config
        self._write_packet(_CHAN_CONTROL, payload)

    def _send_command(self, params: bytes):
        """Send a Command Request (0xF2) on CHAN_CONTROL. params must be 9 bytes."""
        payload = bytes([0xF2, 0x00]) + params  # seq=0, params[0..8]
        self._write_packet(_CHAN_CONTROL, payload)

    def _init(self, rate_hz: int):
        """Power-on init: drain boot packets, enable dynamic calibration, enable rotation vector."""
        self._error = False   # a failed earlier attempt must not block this one
        time.sleep(0.8)   # BNO085 needs 600-800 ms to boot (300 ms too short)
        # Drain advertisement packets sent on CHAN_EXECUTABLE at startup
        for _ in range(15):
            self._read_packet()
            time.sleep(0.02)
        if self._error:
            return   # write already failed during drain — abort
        # Enable dynamic calibration for accel, gyro and magnetometer (cmd 0x07)
        # params: [cmd, accel_cal, gyro_cal, mag_cal, 0, 0, 0, 0, 0]
        self._send_command(bytes([0x07, 1, 1, 1, 0, 0, 0, 0, 0]))
        time.sleep(0.05)
        self._enable_report(_REPORT_ROTATION_VECTOR, rate_hz)
        time.sleep(0.15)
        # Sanity check: can we read back anything?
        channel, payload = self._read_packet()
        if channel is None:
            print("BNO085: no response — check wiring and I2C address")
            self._error = True
        else:
            print(f"BNO085: OK (ch={channel} payload={len(payload)}B)")
            self._error = False

    # ── Data acquisition ──────────────────────────────────────────

    def update(self, max_packets: int = 40) -> bool:
        """Drain pending SHTP packets and keep the latest yaw/pitch/roll.

        The sensor reports at rate_hz while the caller polls slower (50 Hz
        reports, 10 Hz main loop): reading until the queue is empty keeps the
        orientation fresh and stops reports from piling up on the chip.
        max_packets only bounds the time spent here if the bus misbehaves.

        The BNO085 prepends a 5-byte Timestamp Rebase header (0xFB + 4 bytes)
        before each sensor report when running in timestamped mode. We skip
        this prefix before looking for the Rotation Vector report ID (0x05).

        Returns:
            True if yaw/pitch/roll were updated this call.
        """
        updated = False
        for _ in range(max_packets):
            channel, payload = self._read_packet()
            if channel is None:
                break  # no more data queued
            if channel != _CHAN_REPORTS or not payload:
                continue
            # Skip 5-byte Timestamp Rebase prefix (0xFB + 4-byte timestamp)
            offset = 5 if payload[0] == 0xFB and len(payload) > 5 else 0
            if payload[offset] == _REPORT_ROTATION_VECTOR and len(payload) - offset >= 14:
                updated = self._parse_rotation_vector(payload[offset:])
        self._watch_reports(updated)
        return updated

    def _watch_reports(self, updated: bool):
        """Invalidates a frozen orientation, re-initialises a silent sensor."""
        now = time.ticks_ms()
        if updated:
            self._last_report_ms = now
            return
        silent_ms = time.ticks_diff(now, self._last_report_ms)
        if silent_ms > _STALE_MS:
            self.valid = False   # the Pi falls back to the GPS course
        if silent_ms > _REINIT_MS:
            print("BNO085: aucune mesure depuis 5 s, reinitialisation")
            self._init(self._rate_hz)
            self._last_report_ms = time.ticks_ms()

    def _parse_rotation_vector(self, payload: bytes) -> bool:
        # Rotation Vector report layout (after 4-byte SHTP header already stripped):
        #   [0]   report_id (0x05)
        #   [1]   sequence
        #   [2]   status
        #   [3]   delay
        #   [4-5] i  (Q14 int16)
        #   [6-7] j  (Q14 int16)
        #   [8-9] k  (Q14 int16)
        # [10-11] real (Q14 int16)
        # [12-13] accuracy (Q12 uint16, radians)
        i, j, k, real, acc_raw = struct.unpack_from('<hhhhH', payload, 4)
        q14 = 16384.0
        x = i    / q14
        y = j    / q14
        z = k    / q14
        w = real / q14

        # Quaternion → Euler angles (ZYX / aerospace convention)
        # Yaw (heading, around Z axis)
        # BNO085 ENU frame: 0° = East, CCW-positive.
        # Compass convention: 0° = North, CW-positive (N=0, E=90, S=180, W=270).
        # Conversion: heading = (90 - yaw_enu) % 360
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        self.yaw = (90.0 - math.degrees(math.atan2(siny, cosy))) % 360.0

        # Pitch (around Y)
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        self.pitch = math.degrees(math.asin(sinp))

        # Roll (around X)
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        self.roll = math.degrees(math.atan2(sinr, cosr))

        # Accuracy: Q12 radians → degrees
        self.heading_accuracy_deg = math.degrees((acc_raw & 0x7FFF) / 4096.0)
        self.valid = True
        return True

    def save_calibration(self):
        """Persist current calibration to BNO085 internal flash (DCD save, cmd 0x06).

        Waits for the Command Response (0xF1) that confirms the flash write is
        complete before returning. Without this wait the robot could power off
        before the write finishes and the DCD would be corrupted.
        """
        self._send_command(bytes([0x06, 0, 0, 0, 0, 0, 0, 0, 0]))
        # Poll for Command Response on CHAN_CONTROL (up to 2 s)
        deadline = time.ticks_ms() + 2000
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            ch, payload = self._read_packet()
            if (ch == _CHAN_CONTROL and payload and len(payload) >= 3
                    and payload[0] == 0xF1 and payload[2] == 0x06):
                status = payload[3] if len(payload) > 3 else 0
                if status == 0:
                    print("BNO085: calibration saved to flash OK")
                else:
                    print(f"BNO085: DCD save error (status={status})")
                return
            time.sleep(0.01)
        print("BNO085: DCD save sent (no response — check sensor)")

    def is_calibrated(self, threshold_deg: float = 10.0) -> bool:
        """True when heading accuracy is below threshold_deg (lower = better calibrated)."""
        return self.valid and self.heading_accuracy_deg < threshold_deg

    def has_error(self) -> bool:
        return self._error


# ── Standalone test ───────────────────────────────────────────────
if __name__ == '__main__':
    from machine import I2C, Pin

    print("=== BNO085 Test ===")
    # 100 kHz: avoids clock-stretching timeouts during SHTP init on RP2040
    i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=100_000)
    devices = i2c.scan()
    print(f"I2C devices: {[hex(d) for d in devices]}")

    if 0x4A not in devices:
        print("BNO085 not found at 0x4A")
    else:
        imu = BNO085(i2c, addr=0x4A, rate_hz=50)
        print("\nReading (Ctrl+C to stop)...")
        while True:
            updated = imu.update()
            if updated:
                cal = "CAL" if imu.is_calibrated() else f"acc={imu.heading_accuracy_deg:.0f}deg"
                print(f"Yaw {imu.yaw:6.1f}deg  Pitch {imu.pitch:+6.1f}  Roll {imu.roll:+6.1f}  [{cal}]")
            time.sleep(0.02)

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
_CHAN_COMMAND    = 0
_CHAN_EXECUTABLE = 1
_CHAN_CONTROL    = 2  # set-feature commands go here
_CHAN_REPORTS    = 3  # input sensor reports arrive here

# Sensor report IDs
_REPORT_ROTATION_VECTOR = 0x05  # absolute heading (acc+gyr+mag fusion)

# Protocol command
_CMD_SET_FEATURE = 0xFD


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

        self._init(rate_hz)

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

    def _init(self, rate_hz: int):
        """Power-on init: drain boot packets then enable rotation vector."""
        time.sleep(0.8)   # BNO085 needs 600-800 ms to boot (300 ms too short)
        # Drain advertisement packets sent on CHAN_EXECUTABLE at startup
        for _ in range(15):
            self._read_packet()
            time.sleep(0.02)
        if self._error:
            return   # write already failed during drain — abort
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

    def update(self) -> bool:
        """Drain pending SHTP packets and update yaw/pitch/roll.

        Reads up to 8 packets per call to avoid falling behind when the main
        loop runs slower than rate_hz. Stops as soon as a Rotation Vector
        report is parsed (latest value is always the most useful).

        Returns:
            True if yaw/pitch/roll were updated this call.
        """
        for _ in range(8):
            channel, payload = self._read_packet()
            if channel is None:
                break  # no more data queued
            if channel == _CHAN_REPORTS and payload and \
               payload[0] == _REPORT_ROTATION_VECTOR and len(payload) >= 14:
                return self._parse_rotation_vector(payload)
        return False

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
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        self.yaw = math.degrees(math.atan2(siny, cosy)) % 360.0

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

    def is_calibrated(self, threshold_deg: float = 15.0) -> bool:
        """True when heading accuracy is below threshold (sensor well calibrated)."""
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

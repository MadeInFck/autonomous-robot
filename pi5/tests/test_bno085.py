"""Tests for the Pico BNO085 driver: report watchdog (pico/lib, plain Python)."""

import os
import struct
import sys

import pytest

PICO_LIB = os.path.join(os.path.dirname(__file__), '..', '..', 'pico', 'lib')
if not os.path.isdir(PICO_LIB):
    # The Pi only receives pi5/ (the Pico code is flashed from the Mac)
    pytest.skip('pico/lib absent', allow_module_level=True)
sys.path.insert(0, PICO_LIB)

import bno085  # noqa: E402


class FakeTime:
    """MicroPython's time API on a manual clock (sleep advances it)."""

    def __init__(self):
        self.ms = 0

    def ticks_ms(self):
        return self.ms

    def ticks_diff(self, a, b):
        return a - b

    def sleep(self, s):
        self.ms += int(s * 1000)


class FakeI2C:
    """A BNO085 on the bus: serves queued SHTP packets, records writes."""

    def __init__(self):
        self.packets = []
        self.writes = []

    def writeto(self, addr, data):
        self.writes.append(bytes(data))

    def readfrom(self, addr, n):
        if not self.packets:
            return bytes(4)                 # length 0: nothing queued
        if n == 4:
            return self.packets[0][:4]      # header only, packet stays queued
        return self.packets.pop(0)

    def queue_rotation_vector(self, accuracy_rad=0.05):
        report = struct.pack('<BBBBhhhhH', 0x05, 0, 3, 0, 0, 0, 0, 16384,
                             int(accuracy_rad * 4096))
        payload = b'\xfb\x00\x00\x00\x00' + report
        self.packets.append(struct.pack('<HBB', len(payload) + 4, 3, 0) + payload)

    def set_feature_count(self):
        return sum(1 for w in self.writes if len(w) > 4 and w[4] == 0xFD)


@pytest.fixture
def imu(monkeypatch):
    clock = FakeTime()
    monkeypatch.setattr(bno085, 'time', clock)
    i2c = FakeI2C()
    sensor = bno085.BNO085(i2c)
    return sensor, i2c, clock


def tick(sensor, i2c, clock, seconds, reports=True):
    for _ in range(int(seconds * 10)):      # Pico main loop at 10 Hz
        clock.ms += 100
        if reports:
            i2c.queue_rotation_vector()
        sensor.update()


def test_reports_make_the_orientation_valid(imu):
    sensor, i2c, clock = imu
    tick(sensor, i2c, clock, 2)
    assert sensor.valid and sensor.heading_accuracy_deg < 10
    assert i2c.set_feature_count() == 1     # no re-init while reports flow


def test_silent_sensor_is_no_longer_valid(imu):
    """A frozen heading must not keep steering the robot."""
    sensor, i2c, clock = imu
    tick(sensor, i2c, clock, 1)
    tick(sensor, i2c, clock, 1.5, reports=False)
    assert not sensor.valid
    assert i2c.set_feature_count() == 1     # too early to re-init


def test_silent_sensor_is_reinitialised_and_recovers(imu):
    """After a reset the BNO085 sends nothing until its report is re-enabled."""
    sensor, i2c, clock = imu
    tick(sensor, i2c, clock, 1)
    tick(sensor, i2c, clock, 6, reports=False)
    assert i2c.set_feature_count() == 2     # rotation vector enabled again
    tick(sensor, i2c, clock, 1)
    assert sensor.valid


def test_never_reporting_sensor_is_retried_without_hammering(imu):
    sensor, i2c, clock = imu
    tick(sensor, i2c, clock, 30, reports=False)
    # One attempt per ~5 s (plus the time the init itself sleeps), not per loop
    assert 2 <= i2c.set_feature_count() <= 7

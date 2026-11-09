"""Tests for the Pico GPS reader: baud rate detection (pico/lib, plain Python)."""

import os
import sys

import pytest

PICO_LIB = os.path.join(os.path.dirname(__file__), '..', '..', 'pico', 'lib')
if not os.path.isdir(PICO_LIB):
    # The Pi only receives pi5/ (the Pico code is flashed from the Mac)
    pytest.skip('pico/lib absent', allow_module_level=True)
sys.path.insert(0, PICO_LIB)

from gps_reader import GPSReader, BAUD_PROBE_MS  # noqa: E402
from micropyGPS import MicropyGPS  # noqa: E402

# One NEO-6M burst: valid checksums, 8 satellites in use
NMEA = (b'$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n'
        b'$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n')


class FakeModule:
    """A GPS module talking at `module_baud`: garbage when read at another speed."""

    def __init__(self, module_baud):
        self.module_baud = module_baud
        self.opened = []

    def open(self, baud):
        self.opened.append(baud)
        return FakeUart(self, baud)


class FakeUart:
    def __init__(self, module, baud):
        self.module, self.baud = module, baud

    def _burst(self):
        return NMEA if self.baud == self.module.module_baud else bytes(range(0x80, 0xC0))

    def any(self):
        return len(self._burst())

    def read(self, n):
        return self._burst()[:n]


class Clock:
    def __init__(self):
        self.ms = 0

    def __call__(self):
        return self.ms


def make_reader(module_baud):
    module, clock = FakeModule(module_baud), Clock()
    reader = GPSReader(module.open(115200), MicropyGPS(location_formatting='dd'),
                       reopen=module.open, bauds=(115200, 9600), clock=clock)
    return reader, module, clock


def run(reader, clock, seconds):
    for _ in range(seconds * 10):          # main loop at 10 Hz
        clock.ms += 100
        reader.read()


def test_module_back_at_factory_speed_is_found():
    reader, module, clock = make_reader(module_baud=9600)
    run(reader, clock, 5)
    assert reader.baudrate == 9600
    data = reader.read()
    assert data['valid'] and data['satellites'] == 8


def test_right_speed_is_kept():
    reader, module, clock = make_reader(module_baud=115200)
    run(reader, clock, 10)
    assert module.opened == [115200]
    assert reader.read()['valid']


def test_waits_before_switching():
    """A slow start (module still booting) must not be taken for a wrong speed."""
    reader, module, clock = make_reader(module_baud=9600)
    clock.ms = BAUD_PROBE_MS - 100
    reader.read()
    assert module.opened == [115200]


def test_no_reopen_means_no_detection():
    module, clock = FakeModule(9600), Clock()
    reader = GPSReader(module.open(115200), MicropyGPS(), clock=clock)
    run(reader, clock, 10)
    assert module.opened == [115200] and reader.baudrate == 115200

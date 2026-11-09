"""
NEO-6M GPS reader for Raspberry Pi Pico (MicroPython)
Uses micropyGPS to parse NMEA sentences
"""

import time

# MicroPython has wrap-safe ms ticks; plain Python (unit tests) does not
_ticks_ms = getattr(time, 'ticks_ms', lambda: int(time.monotonic() * 1000))
_ticks_diff = getattr(time, 'ticks_diff', lambda a, b: a - b)

# The NEO-6M sends a burst of NMEA sentences every second, fix or not: this
# long without a single valid one means we are listening at the wrong speed.
BAUD_PROBE_MS = 3000


class GPSReader:
    """GPS reader with built-in NMEA parser and baud rate detection.

    The module's serial speed lives in its battery-backed RAM: when that
    backup runs flat (robot left unpowered) it falls back to its factory
    9600 baud while the Pico still listens at 115200, and no sentence ever
    parses — the LED of the module blinks (it has a fix) but no satellite
    reaches the Pi. With `reopen`, the reader cycles through `bauds` until
    sentences with a valid checksum come in, and again if they stop.
    """

    def __init__(self, uart, parser, reopen=None, bauds=(115200, 9600), clock=_ticks_ms):
        """
        Initialize the GPS reader

        Args:
            uart: MicroPython UART instance connected to the GPS, opened at bauds[0]
            parser: MicropyGPS instance for NMEA parsing
            reopen: callable(baudrate) -> UART reopened at that speed; None
                    keeps the given UART as is (no detection)
            bauds: speeds to try, the expected one first
            clock: millisecond tick source (injected by the tests)
        """
        self.uart = uart
        self.parser = parser
        self._reopen = reopen
        self._bauds = bauds
        self._baud_idx = 0
        self.baudrate = bauds[0]
        self._clock = clock
        self._clean = parser.clean_sentences
        self._last_valid = clock()

    def _check_baud(self):
        """Switches to the next baud rate when no valid sentence came for a while."""
        now = self._clock()
        if self.parser.clean_sentences != self._clean:
            self._clean = self.parser.clean_sentences
            self._last_valid = now
            return
        if self._reopen is None or _ticks_diff(now, self._last_valid) < BAUD_PROBE_MS:
            return
        self._baud_idx = (self._baud_idx + 1) % len(self._bauds)
        self.baudrate = self._bauds[self._baud_idx]
        self.uart = self._reopen(self.baudrate)
        self._last_valid = now
        print(f"GPS: aucune trame NMEA valide, essai a {self.baudrate} baud")

    def update(self):
        """
        Feed every byte already received to the NMEA parser (non-blocking)

        Call regularly in the main loop
        """
        pending = self.uart.any()
        data = self.uart.read(pending) if pending else None
        if data:
            for byte in data:
                try:
                    self.parser.update(chr(byte))
                except (TypeError, ValueError):
                    pass
        self._check_baud()

    def _decimal_degrees(self, coord):
        """micropyGPS coordinate ([deg, (min,) hemisphere]) -> signed decimal degrees"""
        if self.parser.coord_format == 'dd':
            value = coord[0]
        else:
            value = coord[0] + coord[1] / 60.0
        if coord[-1] in ('S', 'W'):
            value = -value
        return value

    def read(self):
        """
        Read current GPS data

        Returns:
            dict: {lat, lon, alt, speed, course, satellites, valid}
        """
        self.update()

        if self.has_fix():
            return {
                'lat': self._decimal_degrees(self.parser.latitude),
                'lon': self._decimal_degrees(self.parser.longitude),
                'alt': self.parser.altitude or 0.0,
                # micropyGPS speed is [knots, mph, km/h]
                'speed': self.parser.speed[2] / 3.6 if self.parser.speed else 0.0,
                'course': self.parser.course or 0.0,
                'satellites': self.parser.satellites_in_use or 0,
                'valid': True
            }

        # No fix
        return {
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'speed': 0.0,
            'course': 0.0,
            'satellites': self.parser.satellites_in_view or 0,
            'valid': False
        }

    def has_fix(self):
        """Return True if GPS has a valid fix"""
        return self.parser.valid and self.parser.fix_stat > 0


# Module test
if __name__ == "__main__":
    from machine import UART, Pin
    from micropyGPS import MicropyGPS
    import time

    print("=== Test GPS Reader ===")

    uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5), rxbuf=1024)
    parser = MicropyGPS(local_offset=0, location_formatting='dd')
    gps = GPSReader(uart, parser)

    print("Attente fix GPS (Ctrl+C pour arreter)...")

    while True:
        data = gps.read()
        if data['valid']:
            print(f"FIX: {data['lat']:.6f}, {data['lon']:.6f} | "
                  f"Alt:{data['alt']:.1f}m | Speed:{data['speed']:.1f}m/s | "
                  f"Sat:{data['satellites']}")
        else:
            print(f"NO FIX - Satellites en vue: {data['satellites']}")
        time.sleep(1)

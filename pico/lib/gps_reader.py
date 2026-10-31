"""
NEO-6M GPS reader for Raspberry Pi Pico (MicroPython)
Uses micropyGPS to parse NMEA sentences
"""


class GPSReader:
    """GPS reader with built-in NMEA parser"""

    def __init__(self, uart, parser):
        """
        Initialize the GPS reader

        Args:
            uart: MicroPython UART instance connected to the GPS
            parser: MicropyGPS instance for NMEA parsing
        """
        self.uart = uart
        self.parser = parser

    def update(self):
        """
        Feed every byte already received to the NMEA parser (non-blocking)

        Call regularly in the main loop
        """
        pending = self.uart.any()
        data = self.uart.read(pending) if pending else None
        if not data:
            return
        for byte in data:
            try:
                self.parser.update(chr(byte))
            except (TypeError, ValueError):
                pass

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

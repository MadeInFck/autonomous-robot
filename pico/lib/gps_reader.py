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
        Read and parse available GPS data

        Call regularly in the main loop
        """
        while self.uart.any():
            char = self.uart.read(1)
            if char:
                try:
                    self.parser.update(chr(char[0]))
                except (TypeError, ValueError):
                    pass

    def read(self):
        """
        Read current GPS data

        Returns:
            dict: {lat, lon, alt, speed, course, satellites, valid}
        """
        # Update the parser with available data
        self.update()

        # Check if fix is valid
        if self.parser.valid and self.parser.fix_stat > 0:
            # Latitude in decimal degrees
            lat = self.parser.latitude
            if self.parser.coord_format == 'dd':
                lat_dd = lat[0]
            else:
                lat_dd = lat[0] + lat[1] / 60.0
            if len(lat) > 1 and lat[-1] == 'S':
                lat_dd = -lat_dd

            # Longitude in decimal degrees
            lon = self.parser.longitude
            if self.parser.coord_format == 'dd':
                lon_dd = lon[0]
            else:
                lon_dd = lon[0] + lon[1] / 60.0
            if len(lon) > 1 and lon[-1] == 'W':
                lon_dd = -lon_dd

            # Speed (km/h -> m/s)
            speed_ms = self.parser.speed[2] / 3.6 if self.parser.speed else 0.0

            # Altitude
            altitude = self.parser.altitude if self.parser.altitude else 0.0

            # Course/heading
            course = self.parser.course if self.parser.course else 0.0

            # Satellites
            satellites = self.parser.satellites_in_use if self.parser.satellites_in_use else 0

            return {
                'lat': lat_dd,
                'lon': lon_dd,
                'alt': altitude,
                'speed': speed_ms,
                'course': course,
                'satellites': satellites,
                'valid': True
            }

        # No fix
        return {
            'lat': 0.0,
            'lon': 0.0,
            'alt': 0.0,
            'speed': 0.0,
            'course': 0.0,
            'satellites': self.parser.satellites_in_view if self.parser.satellites_in_view else 0,
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

    uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
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

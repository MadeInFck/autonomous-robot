"""
UART protocol module for Pi Pico (MicroPython)
Handles building BMI160 + GPS frames
Compatible with TestInterfaceUART
"""
import struct

# Protocol constants
STX = 0x02
ETX = 0x03
PAYLOAD_LEN = 30


def calc_crc8(data: bytes) -> int:
    """
    Compute CRC-8 (polynomial 0x07) over the data
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


def build_frame(seq: int,
                acc_x: int, acc_y: int, acc_z: int,
                gyr_x: int, gyr_y: int, gyr_z: int,
                latitude: int, longitude: int,
                altitude: int, vitesse: int, cap: int,
                satellites: int = 0, fix_quality: int = 0) -> bytes:
    """
    Build a complete frame from sensor data

    Args:
        seq: Sequence number (0-65535)
        acc_x, acc_y, acc_z: Accelerometer in mg (-32768 to +32767)
        gyr_x, gyr_y, gyr_z: Gyroscope in 0.1 deg/s (-32768 to +32767)
        latitude: Latitude in microdegrees (int32)
        longitude: Longitude in microdegrees (int32)
        altitude: Altitude in decimeters (int16)
        vitesse: Speed in cm/s (uint16)
        cap: Heading in 0.01 deg (uint16, 0-35999)
        satellites: Number of satellites in use (0-255)
        fix_quality: GPS fix quality (0=no fix, 1=GPS, 2=DGPS)

    Returns:
        Complete frame of 34 bytes
    """
    # Build the payload (little-endian)
    # H = uint16, h = int16, l = int32, B = uint8
    payload = struct.pack('<HhhhhhhllhHHBB',
                          seq,
                          acc_x, acc_y, acc_z,
                          gyr_x, gyr_y, gyr_z,
                          latitude, longitude,
                          altitude, vitesse, cap,
                          satellites, fix_quality)

    # Build LEN + PAYLOAD for CRC
    len_byte = bytes([PAYLOAD_LEN])
    crc_data = len_byte + payload
    crc = calc_crc8(crc_data)

    # Assemble the complete frame
    frame = bytes([STX, PAYLOAD_LEN]) + payload + bytes([crc, ETX])

    return frame


def convert_accel_g_to_mg(g: float) -> int:
    """Convert acceleration from g to mg (millig)"""
    return int(g * 1000)


def convert_gyro_dps_to_raw(dps: float) -> int:
    """Convert rotation from deg/s to raw unit (0.1 deg/s)"""
    return int(dps * 10)


def convert_coord_to_microdeg(deg: float) -> int:
    """Convert decimal degrees to microdegrees"""
    return int(deg * 1_000_000)


def convert_alt_m_to_dm(m: float) -> int:
    """Convert altitude from meters to decimeters"""
    return int(m * 10)


def convert_speed_ms_to_cms(ms: float) -> int:
    """Convert speed from m/s to cm/s"""
    return int(ms * 100)


def convert_heading_to_raw(deg: float) -> int:
    """Convert heading in degrees to raw unit (0.01 deg)"""
    return int(deg * 100) % 36000

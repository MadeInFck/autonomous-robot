"""
Module protocole UART pour Pi Pico (MicroPython)
Gère la construction des trames BMI160 + GPS
Compatible avec TestInterfaceUART
"""
import struct

# Constantes protocole
STX = 0x02
ETX = 0x03
PAYLOAD_LEN = 30


def calc_crc8(data: bytes) -> int:
    """
    Calcule le CRC-8 (polynôme 0x07) sur les données
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
    Construit une trame complète à partir des données capteurs

    Args:
        seq: Numéro de séquence (0-65535)
        acc_x, acc_y, acc_z: Accéléromètre en mg (-32768 à +32767)
        gyr_x, gyr_y, gyr_z: Gyroscope en 0.1°/s (-32768 à +32767)
        latitude: Latitude en microdegrés (int32)
        longitude: Longitude en microdegrés (int32)
        altitude: Altitude en décimètres (int16)
        vitesse: Vitesse en cm/s (uint16)
        cap: Cap en 0.01° (uint16, 0-35999)
        satellites: Nombre de satellites utilisés (0-255)
        fix_quality: Qualité du fix GPS (0=no fix, 1=GPS, 2=DGPS)

    Returns:
        Trame complète de 34 octets
    """
    # Construire le payload (little-endian)
    # H = uint16, h = int16, l = int32, B = uint8
    payload = struct.pack('<HhhhhhhllhHHBB',
                          seq,
                          acc_x, acc_y, acc_z,
                          gyr_x, gyr_y, gyr_z,
                          latitude, longitude,
                          altitude, vitesse, cap,
                          satellites, fix_quality)

    # Construire LEN + PAYLOAD pour le CRC
    len_byte = bytes([PAYLOAD_LEN])
    crc_data = len_byte + payload
    crc = calc_crc8(crc_data)

    # Assembler la trame complète
    frame = bytes([STX, PAYLOAD_LEN]) + payload + bytes([crc, ETX])

    return frame


def convert_accel_g_to_mg(g: float) -> int:
    """Convertit une accélération en g vers mg (millig)"""
    return int(g * 1000)


def convert_gyro_dps_to_raw(dps: float) -> int:
    """Convertit une rotation en °/s vers unité brute (0.1°/s)"""
    return int(dps * 10)


def convert_coord_to_microdeg(deg: float) -> int:
    """Convertit des degrés décimaux en microdegrés"""
    return int(deg * 1_000_000)


def convert_alt_m_to_dm(m: float) -> int:
    """Convertit une altitude en mètres vers décimètres"""
    return int(m * 10)


def convert_speed_ms_to_cms(ms: float) -> int:
    """Convertit une vitesse en m/s vers cm/s"""
    return int(ms * 100)


def convert_heading_to_raw(deg: float) -> int:
    """Convertit un cap en degrés vers unité brute (0.01°)"""
    return int(deg * 100) % 36000

"""
Tests for UART protocol (CRC-8, build_frame, conversions)
Verifies compatibility between the Pico side (build_frame) and Pi5 side (parse_payload)
"""

import struct
import sys
import os
import pytest

# Add paths to import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sensors'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'pico', 'lib'))

from uart_receiver import calc_crc8 as pi5_crc8, parse_payload, PAYLOAD_LEN, STX, ETX
from protocole import (
    calc_crc8 as pico_crc8, build_frame, PAYLOAD_LEN as PICO_PAYLOAD_LEN,
    convert_accel_g_to_mg, convert_gyro_dps_to_raw,
    convert_coord_to_microdeg, convert_alt_m_to_dm,
    convert_speed_ms_to_cms, convert_heading_to_raw,
)


# ===== CRC-8 =====

class TestCRC8:
    def test_crc_empty(self):
        assert pi5_crc8(b'') == 0x00

    def test_crc_known_value(self):
        crc = pi5_crc8(b'\x01\x02\x03')
        assert isinstance(crc, int)
        assert 0 <= crc <= 255

    def test_crc_pico_pi5_identical(self):
        """Both CRC-8 implementations must be identical"""
        test_data = [
            b'',
            b'\x00',
            b'\xFF',
            b'\x01\x02\x03\x04\x05',
            bytes(range(30)),
            b'\xFF' * 31,
        ]
        for data in test_data:
            assert pico_crc8(data) == pi5_crc8(data), f"CRC mismatch pour {data.hex()}"

    def test_crc_different_data_different_result(self):
        assert pi5_crc8(b'\x01') != pi5_crc8(b'\x02')

    def test_crc_single_bit_sensitivity(self):
        """A single bit change must change the CRC"""
        crc1 = pi5_crc8(b'\x00')
        crc2 = pi5_crc8(b'\x01')
        assert crc1 != crc2


# ===== Protocol constants =====

class TestConstantes:
    def test_payload_len_match(self):
        assert PAYLOAD_LEN == PICO_PAYLOAD_LEN == 30

    def test_stx_etx_values(self):
        assert STX == 0x02
        assert ETX == 0x03


# ===== Conversions =====

class TestConversions:
    def test_accel_g_to_mg(self):
        assert convert_accel_g_to_mg(1.0) == 1000
        assert convert_accel_g_to_mg(-1.0) == -1000
        assert convert_accel_g_to_mg(0.0) == 0
        assert convert_accel_g_to_mg(0.981) == 981

    def test_gyro_dps_to_raw(self):
        assert convert_gyro_dps_to_raw(100.0) == 1000
        assert convert_gyro_dps_to_raw(-50.5) == -505
        assert convert_gyro_dps_to_raw(0.0) == 0

    def test_coord_to_microdeg(self):
        assert convert_coord_to_microdeg(48.8566) == 48856600
        assert convert_coord_to_microdeg(-122.4194) == -122419400
        assert convert_coord_to_microdeg(0.0) == 0

    def test_alt_m_to_dm(self):
        assert convert_alt_m_to_dm(100.0) == 1000
        assert convert_alt_m_to_dm(35.2) == 352
        assert convert_alt_m_to_dm(0.0) == 0

    def test_speed_ms_to_cms(self):
        assert convert_speed_ms_to_cms(1.0) == 100
        assert convert_speed_ms_to_cms(0.5) == 50
        assert convert_speed_ms_to_cms(0.0) == 0

    def test_heading_to_raw(self):
        assert convert_heading_to_raw(0.0) == 0
        assert convert_heading_to_raw(180.0) == 18000
        assert convert_heading_to_raw(359.99) == 35999
        # Wrap around
        assert convert_heading_to_raw(360.0) == 0


# ===== build_frame =====

class TestBuildFrame:
    def test_frame_length(self):
        frame = build_frame(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        assert len(frame) == 34

    def test_frame_stx_etx(self):
        frame = build_frame(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        assert frame[0] == STX
        assert frame[-1] == ETX

    def test_frame_len_byte(self):
        frame = build_frame(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        assert frame[1] == PAYLOAD_LEN

    def test_frame_crc_valid(self):
        """The CRC in the frame must match the calculation"""
        frame = build_frame(42, 100, -200, 981, 10, -20, 30,
                            48856600, 2352200, 352, 150, 12750)
        crc_data = frame[1:-2]  # LEN + PAYLOAD
        crc_calculated = pi5_crc8(crc_data)
        crc_in_frame = frame[-2]
        assert crc_in_frame == crc_calculated

    def test_frame_sequence(self):
        frame = build_frame(12345, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        payload = frame[2:-2]
        seq = struct.unpack('<H', payload[0:2])[0]
        assert seq == 12345

    def test_frame_max_sequence(self):
        frame = build_frame(65535, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        payload = frame[2:-2]
        seq = struct.unpack('<H', payload[0:2])[0]
        assert seq == 65535


# ===== Round-trip: build_frame (Pico) → parse_payload (Pi5) =====

class TestRoundTrip:
    def test_zero_values(self):
        frame = build_frame(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        payload = frame[2:-2]
        data = parse_payload(payload)
        assert data['seq'] == 0
        assert data['acc_x'] == 0.0
        assert data['latitude'] == 0.0
        assert data['satellites'] == 0
        assert data['fix_quality'] == 0

    def test_realistic_values(self):
        """Realistic sensor values"""
        # Raw data on Pico side
        acc_x_mg = convert_accel_g_to_mg(0.012)     # 12 mg
        acc_y_mg = convert_accel_g_to_mg(-0.005)     # -5 mg
        acc_z_mg = convert_accel_g_to_mg(0.981)      # 981 mg
        gyr_x = convert_gyro_dps_to_raw(0.1)         # 1
        gyr_y = convert_gyro_dps_to_raw(-0.2)        # -2
        gyr_z = convert_gyro_dps_to_raw(0.05)        # 0 (truncated)
        lat = convert_coord_to_microdeg(48.8566)     # Paris
        lon = convert_coord_to_microdeg(2.3522)
        alt = convert_alt_m_to_dm(35.2)
        speed = convert_speed_ms_to_cms(1.5)
        heading = convert_heading_to_raw(127.5)

        frame = build_frame(
            seq=1234,
            acc_x=acc_x_mg, acc_y=acc_y_mg, acc_z=acc_z_mg,
            gyr_x=gyr_x, gyr_y=gyr_y, gyr_z=gyr_z,
            latitude=lat, longitude=lon,
            altitude=alt, vitesse=speed, cap=heading,
            satellites=9, fix_quality=1
        )

        payload = frame[2:-2]
        data = parse_payload(payload)

        assert data['seq'] == 1234
        assert abs(data['acc_x'] - 0.012) < 0.001
        assert abs(data['acc_y'] - (-0.005)) < 0.001
        assert abs(data['acc_z'] - 0.981) < 0.001
        assert abs(data['latitude'] - 48.8566) < 0.000001
        assert abs(data['longitude'] - 2.3522) < 0.000001
        assert abs(data['altitude'] - 35.2) < 0.1
        assert abs(data['vitesse_ms'] - 1.5) < 0.01
        assert abs(data['cap'] - 127.5) < 0.01
        assert data['satellites'] == 9
        assert data['fix_quality'] == 1

    def test_negative_coordinates(self):
        """Negative coordinates (southern/western hemisphere)"""
        lat = convert_coord_to_microdeg(-33.8688)  # Sydney
        lon = convert_coord_to_microdeg(151.2093)

        frame = build_frame(0, 0, 0, 0, 0, 0, 0, lat, lon, 0, 0, 0)
        payload = frame[2:-2]
        data = parse_payload(payload)

        assert abs(data['latitude'] - (-33.8688)) < 0.000001
        assert abs(data['longitude'] - 151.2093) < 0.000001

    def test_gps_no_fix(self):
        """GPS without fix: fix_quality = 0"""
        frame = build_frame(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            satellites=4, fix_quality=0)
        payload = frame[2:-2]
        data = parse_payload(payload)
        assert data['satellites'] == 4
        assert data['fix_quality'] == 0

    def test_max_speed(self):
        """Maximum speed (65535 cm/s = 655.35 m/s)"""
        frame = build_frame(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 65535, 0)
        payload = frame[2:-2]
        data = parse_payload(payload)
        assert abs(data['vitesse_ms'] - 655.35) < 0.01

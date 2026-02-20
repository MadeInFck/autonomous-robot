"""
Tests for FrameReader (state machine) and UART decoding
"""

import sys
import os
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sensors'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'pico', 'lib'))

from uart_receiver import FrameReader, parse_payload, calc_crc8, STX, ETX, PAYLOAD_LEN
from protocole import build_frame


def make_valid_frame(**kwargs):
    """Helper: build a valid frame with default values"""
    defaults = dict(
        seq=1, acc_x=0, acc_y=0, acc_z=981,
        gyr_x=0, gyr_y=0, gyr_z=0,
        latitude=48856600, longitude=2352200,
        altitude=352, vitesse=0, cap=0,
        satellites=8, fix_quality=1
    )
    defaults.update(kwargs)
    return build_frame(**defaults)


# ===== FrameReader: valid frames =====

class TestFrameReaderValid:
    def test_single_valid_frame(self):
        reader = FrameReader()
        frame = make_valid_frame()
        reader.feed(frame)

        result = reader.get_frame()
        assert result is not None
        assert result['crc_ok'] is True
        assert result['seq'] == 1

    def test_multiple_frames(self):
        reader = FrameReader()
        for i in range(5):
            frame = make_valid_frame(seq=i)
            reader.feed(frame)

        for i in range(5):
            result = reader.get_frame()
            assert result is not None
            assert result['seq'] == i

        assert reader.get_frame() is None

    def test_frame_data_decoded(self):
        reader = FrameReader()
        frame = make_valid_frame(
            seq=42, acc_x=100, acc_y=-200, acc_z=981,
            satellites=12, fix_quality=2
        )
        reader.feed(frame)

        result = reader.get_frame()
        assert result['seq'] == 42
        assert abs(result['acc_x'] - 0.1) < 0.001
        assert abs(result['acc_y'] - (-0.2)) < 0.001
        assert result['satellites'] == 12
        assert result['fix_quality'] == 2

    def test_byte_by_byte(self):
        """Byte-by-byte feeding (as in serial reception)"""
        reader = FrameReader()
        frame = make_valid_frame()

        for byte in frame:
            reader.feed(bytes([byte]))

        result = reader.get_frame()
        assert result is not None
        assert result['crc_ok'] is True


# ===== FrameReader: errors =====

class TestFrameReaderErrors:
    def test_bad_crc(self):
        reader = FrameReader()
        frame = bytearray(make_valid_frame())
        frame[-2] ^= 0xFF  # Corrupt the CRC
        reader.feed(bytes(frame))

        assert reader.get_frame() is None
        error = reader.get_error()
        assert error is not None
        assert "CRC" in error

    def test_missing_etx(self):
        reader = FrameReader()
        frame = bytearray(make_valid_frame())
        frame[-1] = 0x00  # Replace ETX with something else
        reader.feed(bytes(frame))

        assert reader.get_frame() is None
        error = reader.get_error()
        assert error is not None
        assert "ETX" in error

    def test_garbage_before_frame(self):
        """Random bytes before STX should be ignored"""
        reader = FrameReader()
        garbage = bytes([0x55, 0xAA, 0xFF, 0x00, 0x42])
        frame = make_valid_frame(seq=99)
        reader.feed(garbage + frame)

        result = reader.get_frame()
        assert result is not None
        assert result['seq'] == 99

    def test_garbage_between_frames(self):
        """Garbage between two valid frames"""
        reader = FrameReader()
        f1 = make_valid_frame(seq=1)
        f2 = make_valid_frame(seq=2)
        garbage = bytes([0xFF, 0xAA, 0x55])

        reader.feed(f1 + garbage + f2)

        r1 = reader.get_frame()
        assert r1 is not None
        assert r1['seq'] == 1

        r2 = reader.get_frame()
        assert r2 is not None
        assert r2['seq'] == 2

    def test_partial_frame_then_complete(self):
        """Partial frame followed by a complete frame"""
        reader = FrameReader()
        frame = make_valid_frame(seq=77)

        # Send the first half
        reader.feed(frame[:17])
        assert reader.get_frame() is None

        # Send the rest
        reader.feed(frame[17:])
        result = reader.get_frame()
        assert result is not None
        assert result['seq'] == 77

    def test_zero_length(self):
        """Length 0 must be rejected"""
        reader = FrameReader()
        reader.feed(bytes([STX, 0x00]))
        error = reader.get_error()
        assert error is not None
        assert "invalide" in error.lower()

    def test_corrupted_payload(self):
        """Corrupted payload but CRC preserved = CRC mismatch"""
        reader = FrameReader()
        frame = bytearray(make_valid_frame())
        frame[10] ^= 0xFF  # Corrupt a payload byte
        reader.feed(bytes(frame))

        assert reader.get_frame() is None
        assert reader.get_error() is not None


# ===== parse_payload =====

class TestParsePayload:
    def test_payload_size(self):
        """The payload must be exactly 30 bytes"""
        import struct
        payload = struct.pack('<HhhhhhhllhHHBB',
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        assert len(payload) == PAYLOAD_LEN

    def test_speed_conversion(self):
        """Speed cm/s -> m/s and km/h"""
        import struct
        # 500 cm/s = 5.0 m/s = 18.0 km/h
        payload = struct.pack('<HhhhhhhllhHHBB',
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 500, 0, 0, 0)
        data = parse_payload(payload)
        assert abs(data['vitesse_ms'] - 5.0) < 0.01
        assert abs(data['vitesse_kmh'] - 18.0) < 0.01

    def test_heading_conversion(self):
        """Heading 0.01deg -> deg"""
        import struct
        # 27050 = 270.50 deg
        payload = struct.pack('<HhhhhhhllhHHBB',
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27050, 0, 0)
        data = parse_payload(payload)
        assert abs(data['cap'] - 270.5) < 0.01


# ===== Sequence checking =====

class TestSequenceCheck:
    def test_sequence_wrap_around(self):
        """Sequence must wrap from 65535 to 0"""
        frame_max = make_valid_frame(seq=65535)
        frame_zero = make_valid_frame(seq=0)

        reader = FrameReader()
        reader.feed(frame_max)
        reader.feed(frame_zero)

        r1 = reader.get_frame()
        r2 = reader.get_frame()
        assert r1['seq'] == 65535
        assert r2['seq'] == 0

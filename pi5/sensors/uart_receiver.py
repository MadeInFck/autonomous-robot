"""
UART Receiver module for Raspberry Pi 5
Receives sensor data from the Pico via UART3

Port: /dev/ttyAMA3 (GPIO8=TX, GPIO9=RX)
Protocol: STX/ETX with CRC-8 (compatible with TestInterfaceUART)
"""

import struct
import time
import threading
import json
import logging
import os
import serial
from collections import deque
from dataclasses import dataclass
from typing import Optional
from enum import Enum, auto

logger = logging.getLogger(__name__)

# Path where BNO085 calibration state is persisted across reboots
_CAL_STATE_PATH = os.path.expanduser('~/.config/robot/imu_cal_state.json')

# BNO085 heading accuracy thresholds (degrees)
_CAL_OK_DEG = 10.0       # below: calibrated (same threshold as BNO085.is_calibrated on the Pico)
_CAL_LOST_DEG = 30.0     # above: strong magnetic disturbance, forget the persisted calibration

# Protocol constants (must match pico/lib/protocole.py — checked by test_protocole)
STX = 0x02
ETX = 0x03
PAYLOAD_LEN = 30
FRAME_LEN = 34  # STX + LEN + PAYLOAD + CRC + ETX

# fix_quality byte layout:
#   bits 0-3: GPS fix quality (0=no fix, 1=GPS, 2=DGPS)
#   bit 7:    IMU is BNO085 — acc_x/acc_y carry pitch/roll (0.01 deg),
#             gyr_z carries heading_accuracy (0.1 deg)
FIX_QUALITY_MASK = 0x0F
FLAG_IMU_BNO085 = 0x80


class State(Enum):
    """Reception state machine states"""
    IDLE = auto()
    GET_LEN = auto()
    GET_DATA = auto()
    CHECK_ETX = auto()


@dataclass
class SensorData:
    """Sensor data structure received from the Pico"""
    # Sequence
    sequence: int = 0

    # BMI160 Accelerometer (g) — 0 with a BNO085
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0

    # BMI160 Gyroscope (°/s) — 0 with a BNO085
    gyr_x: float = 0.0
    gyr_y: float = 0.0
    gyr_z: float = 0.0

    # GPS
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0       # meters
    speed: float = 0.0          # m/s
    speed_kmh: float = 0.0      # km/h
    heading: float = 0.0        # degrees (BNO085 yaw, or GPS course with a BMI160)
    satellites: int = 0
    has_fix: bool = False

    # IMU type and calibration (BNO085 only)
    imu_type: str = 'unknown'              # 'BNO085' or 'BMI160'
    heading_accuracy_deg: float = 180.0    # BNO085: lower = better (< 10 = calibrated)
    imu_calibrated: bool = False

    # Orientation (BNO085: from sensor fusion; BMI160: 0.0)
    pitch: float = 0.0  # degrees
    roll: float = 0.0   # degrees

    # Timestamp
    timestamp: float = 0.0


def calc_crc8(data: bytes) -> int:
    """
    Computes CRC-8 (polynomial 0x07) over the data
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


def parse_payload(payload: bytes) -> dict:
    """
    Decodes the payload into sensor data

    Args:
        payload: 30 bytes of data (PAYLOAD_LEN)

    Returns:
        Dict with all decoded data. With a BNO085 (bit 7 of fix_quality) it
        also carries pitch, roll and heading_accuracy_deg, decoded from the
        BMI160 fields the BNO085 repurposes.
    """
    # H = uint16, h = int16, i = int32, B = uint8
    (seq,
     acc_x, acc_y, acc_z,
     gyr_x, gyr_y, gyr_z,
     latitude, longitude,
     altitude, vitesse, cap,
     satellites, fix_quality) = struct.unpack('<H hhh hhh ii hHH BB', payload)

    data = {
        'seq': seq,
        # BMI160 - Accelerometer (mg -> g)
        'acc_x': acc_x / 1000.0,
        'acc_y': acc_y / 1000.0,
        'acc_z': acc_z / 1000.0,
        # BMI160 - Gyroscope (0.1°/s → °/s)
        'gyr_x': gyr_x / 10.0,
        'gyr_y': gyr_y / 10.0,
        'gyr_z': gyr_z / 10.0,
        # GPS - Position (microdegrees -> degrees)
        'latitude': latitude / 1_000_000.0,
        'longitude': longitude / 1_000_000.0,
        # GPS - Altitude (decimeters -> meters)
        'altitude': altitude / 10.0,
        # GPS - Speed (cm/s -> m/s and km/h)
        'vitesse_ms': vitesse / 100.0,
        'vitesse_kmh': vitesse * 0.036,
        # Heading (0.01deg -> degrees)
        'cap': cap / 100.0,
        # GPS - Satellites and fix
        'satellites': satellites,
        'fix_quality': fix_quality & FIX_QUALITY_MASK,
        # Flags (high bits of the fix_quality byte)
        'imu_is_bno085': bool(fix_quality & FLAG_IMU_BNO085),
    }
    if data['imu_is_bno085']:
        data['pitch'] = acc_x / 100.0                 # 0.01 deg -> deg
        data['roll'] = acc_y / 100.0
        data['heading_accuracy_deg'] = gyr_z / 10.0   # 0.1 deg -> deg
    return data


class FrameReader:
    """
    State machine for reading UART frames byte by byte
    """

    def __init__(self):
        self.state = State.IDLE
        self.length = 0
        self.buffer = bytearray()
        self.frames = deque()  # Decoded complete frames
        self.errors = deque()  # Detected errors

    def feed(self, data: bytes):
        """
        Feeds the state machine with received bytes

        Args:
            data: Bytes received from the serial port
        """
        for byte in data:
            self._process_byte(byte)

    def _process_byte(self, byte: int):
        """Processes a byte according to the current state"""

        if self.state == State.IDLE:
            if byte == STX:
                self.state = State.GET_LEN
                self.buffer = bytearray()

        elif self.state == State.GET_LEN:
            # The payload size is fixed: anything else is line noise (an STX
            # byte inside garbage). Rejecting it at once resyncs on the next
            # real frame instead of swallowing up to 255 bytes of good frames.
            self.length = byte
            self.buffer.append(byte)
            if self.length == PAYLOAD_LEN:
                self.state = State.GET_DATA
            else:
                self._error(f"Longueur invalide ({self.length})")
                self.state = State.IDLE

        elif self.state == State.GET_DATA:
            self.buffer.append(byte)
            # Expect LEN (payload) + 1 (CRC) bytes after the LEN byte
            if len(self.buffer) == 1 + self.length + 1:
                self.state = State.CHECK_ETX

        elif self.state == State.CHECK_ETX:
            if byte == ETX:
                self._validate_frame()
            else:
                self._error("ETX manquant")
            self.state = State.IDLE

    def _validate_frame(self):
        """Validates the CRC and extracts the data"""
        # buffer = [LEN, PAYLOAD..., CRC]
        crc_received = self.buffer[-1]
        crc_calculated = calc_crc8(bytes(self.buffer[:-1]))  # over LEN + PAYLOAD

        if crc_received == crc_calculated:
            data = parse_payload(bytes(self.buffer[1:-1]))
            data['crc_ok'] = True
            self.frames.append(data)
        else:
            self._error(f"CRC invalide (recu={crc_received:02X}, calcule={crc_calculated:02X})")

    def _error(self, message: str):
        """Records an error"""
        self.errors.append(message)

    def get_frame(self) -> Optional[dict]:
        """Retrieves a decoded frame (FIFO), or None if no frame is available"""
        return self.frames.popleft() if self.frames else None

    def get_error(self) -> Optional[str]:
        """Retrieves an error message (FIFO), or None"""
        return self.errors.popleft() if self.errors else None


def _load_cal_state(path: str) -> bool:
    """True if a valid BNO085 calibration was saved in a previous session."""
    try:
        with open(path) as f:
            return bool(json.load(f).get('calibrated', False))
    except Exception:
        return False


def _save_cal_state(path: str, calibrated: bool):
    """Persists the BNO085 calibration state to disk (best effort)."""
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            json.dump({'calibrated': calibrated, 'timestamp': time.time()}, f)
    except Exception:
        pass


class ImuCalibrationLatch:
    """Decides whether the BNO085 heading is trustworthy, across reboots.

    The Pico saves the sensor's calibration (DCD) to its flash once it is
    reached; after a reboot the BNO085 reloads it but its reported accuracy
    takes a while to converge again. Persisting "a calibration exists" lets
    the robot (and the UI) trust the heading immediately, until a strong
    magnetic disturbance (accuracy > _CAL_LOST_DEG) proves otherwise.
    """

    def __init__(self, path: str = _CAL_STATE_PATH):
        self._path = path
        self._persisted = _load_cal_state(path)
        self._saved = self._persisted  # avoid rewriting a state already on disk

    def update(self, heading_accuracy_deg: float) -> bool:
        """Feeds one accuracy reading; returns whether the heading is calibrated."""
        calibrated = heading_accuracy_deg < _CAL_OK_DEG or self._persisted
        if calibrated and not self._saved:
            _save_cal_state(self._path, True)
            self._saved = True
        elif heading_accuracy_deg > _CAL_LOST_DEG and self._persisted:
            self._persisted = False
            self._saved = False
            _save_cal_state(self._path, False)
        return calibrated


def frame_to_sensor_data(frame: dict, calibration: ImuCalibrationLatch) -> SensorData:
    """Converts a decoded frame into SensorData, timestamped now."""
    common = dict(
        sequence=frame['seq'],
        latitude=frame['latitude'],
        longitude=frame['longitude'],
        altitude=frame['altitude'],
        speed=frame['vitesse_ms'],
        speed_kmh=frame['vitesse_kmh'],
        heading=frame['cap'],
        satellites=frame['satellites'],
        has_fix=frame['fix_quality'] > 0,
        timestamp=time.time(),
    )
    if frame['imu_is_bno085']:
        accuracy = frame['heading_accuracy_deg']
        return SensorData(
            imu_type='BNO085',
            heading_accuracy_deg=accuracy,
            imu_calibrated=calibration.update(accuracy),
            pitch=frame['pitch'],
            roll=frame['roll'],
            **common,
        )
    return SensorData(
        acc_x=frame['acc_x'], acc_y=frame['acc_y'], acc_z=frame['acc_z'],
        gyr_x=frame['gyr_x'], gyr_y=frame['gyr_y'], gyr_z=frame['gyr_z'],
        imu_type='BMI160',
        **common,
    )


class UARTReceiver:
    """
    UART receiver for sensor data from the Pico

    Protocol: STX/ETX with CRC-8, 34 bytes per frame
    Port: /dev/ttyAMA3 (UART3 on GPIO8/9)
    """

    def __init__(self, port: str = '/dev/ttyAMA3', baudrate: int = 115200):
        """
        Initializes the UART receiver

        Args:
            port: Serial port (default: /dev/ttyAMA3 for Pi5 UART3)
            baudrate: Baud rate (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self._running = False
        self._thread = None
        self._reader = FrameReader()

        # Statistics
        self._packets_received = 0
        self._packets_lost = 0
        self._errors = 0
        self._last_sequence = -1

        # Last received data
        self._last_data = None
        self._lock = threading.Lock()

    def open(self) -> bool:
        """
        Opens the serial connection

        Returns:
            bool: True if successful
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            logger.info(f"UART ouvert: {self.port} @ {self.baudrate} baud")
            return True

        except Exception as e:
            logger.error(f"Erreur ouverture UART: {e}")
            return False

    def close(self):
        """Closes the serial connection"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None

    def start_async(self):
        """Starts reception in a background thread (read results with get_last_data)"""
        if not self.serial and not self.open():
            return

        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        logger.info("Réception UART démarrée (async)")

    def _receive_loop(self):
        """Reception loop (thread)"""
        # Restores the calibration state of the previous session, so the UI
        # shows "Calibré" right after a reboot when a DCD was saved before.
        calibration = ImuCalibrationLatch()

        while self._running:
            try:
                # Blocks until a byte arrives (or the 0.1 s port timeout), then
                # takes whatever else is already buffered: wakes up per frame
                # instead of polling.
                data = self.serial.read(self.serial.in_waiting or 1)
                if not data:
                    continue
                self._reader.feed(data)

                while (frame := self._reader.get_frame()) is not None:
                    self._check_sequence(frame['seq'])
                    self._packets_received += 1
                    sensor_data = frame_to_sensor_data(frame, calibration)
                    with self._lock:
                        self._last_data = sensor_data

                while self._reader.get_error() is not None:
                    self._errors += 1

            except Exception as e:
                logger.error(f"Erreur réception: {e}")
                time.sleep(0.1)

    def _check_sequence(self, current_seq: int):
        """Checks the sequence counter to detect lost frames"""
        if self._last_sequence >= 0:
            expected = (self._last_sequence + 1) % 65536
            if current_seq != expected:
                self._packets_lost += (current_seq - expected) % 65536
        self._last_sequence = current_seq

    def get_last_data(self) -> Optional[SensorData]:
        """Returns the last received data (thread-safe)"""
        with self._lock:
            return self._last_data

    def get_stats(self) -> dict:
        """Returns communication statistics"""
        total = self._packets_received + self._packets_lost
        loss_rate = (self._packets_lost / total * 100) if total > 0 else 0

        return {
            'packets_received': self._packets_received,
            'packets_lost': self._packets_lost,
            'errors': self._errors,
            'loss_rate': f"{loss_rate:.2f}%"
        }


# Module test
if __name__ == "__main__":
    print("=" * 60)
    print("  Test UART Receiver - Pi 5")
    print("  Port: /dev/ttyAMA3 (UART3 GPIO8/9)")
    print("  Protocole: STX/ETX + CRC8")
    print("=" * 60)

    receiver = UARTReceiver(port='/dev/ttyAMA3', baudrate=115200)

    if receiver.open():
        print("Attente de données (Ctrl+C pour arrêter)...\n")

        try:
            receiver.start_async()
            last_seq = None
            while True:
                time.sleep(1.0)
                data = receiver.get_last_data()
                if data is None or data.sequence == last_seq:
                    print("Timeout - pas de donnees")
                    continue
                last_seq = data.sequence
                print(f"[Seq {data.sequence:5d}] IMU {data.imu_type}")
                if data.imu_type == 'BNO085':
                    print(f"  Cap={data.heading:.1f}deg Pitch={data.pitch:+.1f} Roll={data.roll:+.1f}"
                          f" Precision={data.heading_accuracy_deg:.1f}deg"
                          f" {'CAL' if data.imu_calibrated else 'non calibre'}")
                else:
                    print(f"  Accel: X={data.acc_x:+.3f}g Y={data.acc_y:+.3f}g Z={data.acc_z:+.3f}g")
                    print(f"  Gyro:  X={data.gyr_x:+.1f}deg/s Y={data.gyr_y:+.1f}deg/s Z={data.gyr_z:+.1f}deg/s")
                fix_str = "FIX" if data.has_fix else "NO FIX"
                print(f"  GPS:   {data.latitude:.6f}deg, {data.longitude:.6f}deg [{fix_str} {data.satellites}sat]")
                print(f"         Alt={data.altitude:.1f}m Speed={data.speed_kmh:.1f}km/h")
                print()

        except KeyboardInterrupt:
            print("\nArret demande")

        print("\nStatistiques:")
        for k, v in receiver.get_stats().items():
            print(f"  {k}: {v}")

        receiver.close()
    else:
        print("Impossible d'ouvrir le port serie")
        print("\nVerifiez:")
        print("  1. dtoverlay=uart3-pi5 dans /boot/firmware/config.txt")
        print("  2. Cablage: GPIO8->GP1, GPIO9<-GP0, GND")

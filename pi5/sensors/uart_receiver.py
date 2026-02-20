"""
UART Receiver module for Raspberry Pi 5
Receives sensor data from the Pico via UART3

Port: /dev/ttyAMA3 (GPIO8=TX, GPIO9=RX)
Protocol: STX/ETX with CRC-8 (compatible with TestInterfaceUART)
"""

import struct
import time
import threading
import serial
from dataclasses import dataclass
from typing import Optional, Callable
from queue import Queue, Empty
from enum import Enum, auto


# Protocol constants
STX = 0x02
ETX = 0x03
PAYLOAD_LEN = 30
FRAME_LEN = 34  # STX + LEN + PAYLOAD + CRC + ETX


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

    # BMI160 Accelerometer (g)
    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0

    # BMI160 Gyroscope (°/s)
    gyr_x: float = 0.0
    gyr_y: float = 0.0
    gyr_z: float = 0.0

    # GPS
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0       # meters
    speed: float = 0.0          # m/s
    speed_kmh: float = 0.0      # km/h
    heading: float = 0.0        # degrees
    satellites: int = 0
    has_fix: bool = False

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
        payload: 28 bytes of data

    Returns:
        Dict with all decoded data
    """
    # H = uint16, h = int16, i = int32, B = uint8
    (seq,
     acc_x, acc_y, acc_z,
     gyr_x, gyr_y, gyr_z,
     latitude, longitude,
     altitude, vitesse, cap,
     satellites, fix_quality) = struct.unpack('<H hhh hhh ii hHH BB', payload)

    return {
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
        # GPS - Heading (0.01deg -> degrees)
        'cap': cap / 100.0,
        # GPS - Satellites and fix
        'satellites': satellites,
        'fix_quality': fix_quality,
    }


class FrameReader:
    """
    State machine for reading UART frames byte by byte
    """

    def __init__(self):
        self.state = State.IDLE
        self.length = 0
        self.buffer = bytearray()
        self.frames = []  # Decoded complete frames
        self.errors = []  # Detected errors

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
            self.length = byte
            self.buffer.append(byte)
            if 0 < self.length <= 255:
                self.state = State.GET_DATA
            else:
                self._error("Longueur invalide")
                self.state = State.IDLE

        elif self.state == State.GET_DATA:
            self.buffer.append(byte)
            # On attend LEN (payload) + 1 (CRC) octets
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
        crc_data = bytes(self.buffer[:-1])  # LEN + PAYLOAD
        crc_received = self.buffer[-1]
        crc_calculated = calc_crc8(crc_data)

        if crc_received == crc_calculated:
            payload = bytes(self.buffer[1:-1])  # Extraire PAYLOAD
            if len(payload) == PAYLOAD_LEN:
                data = parse_payload(payload)
                data['crc_ok'] = True
                self.frames.append(data)
            else:
                self._error(f"Taille payload invalide ({len(payload)} != {PAYLOAD_LEN})")
        else:
            self._error(f"CRC invalide (recu={crc_received:02X}, calcule={crc_calculated:02X})")

    def _error(self, message: str):
        """Records an error"""
        self.errors.append(message)

    def get_frame(self) -> Optional[dict]:
        """
        Retrieves a decoded frame (FIFO)

        Returns:
            Dict with data or None if no frame available
        """
        if self.frames:
            return self.frames.pop(0)
        return None

    def get_error(self) -> Optional[str]:
        """
        Retrieves an error (FIFO)

        Returns:
            Error message or None
        """
        if self.errors:
            return self.errors.pop(0)
        return None


class UARTReceiver:
    """
    UART receiver for sensor data from the Pico

    Protocol: STX/ETX with CRC-8, 32 bytes per frame
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
        self._data_queue = Queue(maxsize=10)
        self._callback = None
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
            print(f"UART ouvert: {self.port} @ {self.baudrate} baud")
            return True

        except Exception as e:
            print(f"Erreur ouverture UART: {e}")
            return False

    def close(self):
        """Closes the serial connection"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None

    def start_async(self, callback: Optional[Callable[[SensorData], None]] = None):
        """
        Starts reception in asynchronous mode (thread)

        Args:
            callback: Function called for each received packet
        """
        if not self.serial:
            if not self.open():
                return

        self._callback = callback
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()
        print("Réception UART démarrée (async)")

    def _receive_loop(self):
        """Reception loop (thread)"""
        while self._running:
            try:
                # Read available bytes
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self._reader.feed(data)

                # Process decoded frames
                while True:
                    frame = self._reader.get_frame()
                    if frame is None:
                        break

                    # Detect lost frames
                    self._check_sequence(frame['seq'])
                    self._packets_received += 1

                    # Convert to SensorData
                    sensor_data = SensorData(
                        sequence=frame['seq'],
                        acc_x=frame['acc_x'],
                        acc_y=frame['acc_y'],
                        acc_z=frame['acc_z'],
                        gyr_x=frame['gyr_x'],
                        gyr_y=frame['gyr_y'],
                        gyr_z=frame['gyr_z'],
                        latitude=frame['latitude'],
                        longitude=frame['longitude'],
                        altitude=frame['altitude'],
                        speed=frame['vitesse_ms'],
                        speed_kmh=frame['vitesse_kmh'],
                        heading=frame['cap'],
                        satellites=frame['satellites'],
                        has_fix=frame['fix_quality'] > 0,
                        timestamp=time.time()
                    )

                    # Store latest data
                    with self._lock:
                        self._last_data = sensor_data

                    # Callback or queue
                    if self._callback:
                        self._callback(sensor_data)
                    else:
                        try:
                            self._data_queue.put_nowait(sensor_data)
                        except:
                            pass  # Queue pleine

                # Process errors
                while True:
                    error = self._reader.get_error()
                    if error is None:
                        break
                    self._errors += 1

                time.sleep(0.01)

            except Exception as e:
                print(f"Erreur réception: {e}")
                time.sleep(0.1)

    def _check_sequence(self, current_seq: int):
        """Checks the sequence counter to detect lost frames"""
        if self._last_sequence >= 0:
            expected = (self._last_sequence + 1) % 65536
            if current_seq != expected:
                lost = (current_seq - expected) % 65536
                self._packets_lost += lost
        self._last_sequence = current_seq

    def read_sensor_data(self, timeout: float = 1.0) -> Optional[SensorData]:
        """
        Reads sensor data

        Args:
            timeout: Timeout in seconds

        Returns:
            SensorData or None
        """
        if self._running:
            try:
                return self._data_queue.get(timeout=timeout)
            except Empty:
                return None
        return None

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

            while True:
                data = receiver.read_sensor_data(timeout=1.0)
                if data:
                    print(f"[Seq {data.sequence:5d}]")
                    print(f"  Accel: X={data.acc_x:+.3f}g Y={data.acc_y:+.3f}g Z={data.acc_z:+.3f}g")
                    print(f"  Gyro:  X={data.gyr_x:+.1f}deg/s Y={data.gyr_y:+.1f}deg/s Z={data.gyr_z:+.1f}deg/s")
                    fix_str = "FIX" if data.has_fix else "NO FIX"
                    print(f"  GPS:   {data.latitude:.6f}deg, {data.longitude:.6f}deg [{fix_str} {data.satellites}sat]")
                    print(f"         Alt={data.altitude:.1f}m Speed={data.speed_kmh:.1f}km/h Cap={data.heading:.1f}deg")
                    print()
                else:
                    print("Timeout - pas de donnees")

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

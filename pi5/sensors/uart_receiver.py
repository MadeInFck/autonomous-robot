"""
Module UART Receiver pour Raspberry Pi 5
Réception des données capteurs depuis le Pico via UART3

Port: /dev/ttyAMA3 (GPIO8=TX, GPIO9=RX)
Protocole: STX/ETX avec CRC-8 (compatible TestInterfaceUART)
"""

import struct
import time
import threading
import serial
from dataclasses import dataclass
from typing import Optional, Callable
from queue import Queue, Empty
from enum import Enum, auto


# Constantes protocole
STX = 0x02
ETX = 0x03
PAYLOAD_LEN = 28
FRAME_LEN = 32  # STX + LEN + PAYLOAD + CRC + ETX


class State(Enum):
    """États de la machine à états de réception"""
    IDLE = auto()
    GET_LEN = auto()
    GET_DATA = auto()
    CHECK_ETX = auto()


@dataclass
class SensorData:
    """Structure des données capteurs reçues du Pico"""
    # Séquence
    sequence: int = 0

    # BMI160 Accéléromètre (g)
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
    altitude: float = 0.0       # mètres
    speed: float = 0.0          # m/s
    speed_kmh: float = 0.0      # km/h
    heading: float = 0.0        # degrés

    # Timestamp
    timestamp: float = 0.0


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


def parse_payload(payload: bytes) -> dict:
    """
    Décode le payload en données capteurs

    Args:
        payload: 28 octets de données

    Returns:
        Dict avec toutes les données décodées
    """
    # H = uint16, h = int16, i = int32
    (seq,
     acc_x, acc_y, acc_z,
     gyr_x, gyr_y, gyr_z,
     latitude, longitude,
     altitude, vitesse, cap) = struct.unpack('<H hhh hhh ii hHH', payload)

    return {
        'seq': seq,
        # BMI160 - Accéléromètre (mg → g)
        'acc_x': acc_x / 1000.0,
        'acc_y': acc_y / 1000.0,
        'acc_z': acc_z / 1000.0,
        # BMI160 - Gyroscope (0.1°/s → °/s)
        'gyr_x': gyr_x / 10.0,
        'gyr_y': gyr_y / 10.0,
        'gyr_z': gyr_z / 10.0,
        # GPS - Position (microdegrés → degrés)
        'latitude': latitude / 1_000_000.0,
        'longitude': longitude / 1_000_000.0,
        # GPS - Altitude (décimètres → mètres)
        'altitude': altitude / 10.0,
        # GPS - Vitesse (cm/s → m/s et km/h)
        'vitesse_ms': vitesse / 100.0,
        'vitesse_kmh': vitesse * 0.036,
        # GPS - Cap (0.01° → degrés)
        'cap': cap / 100.0,
    }


class FrameReader:
    """
    Machine à états pour lire les trames UART octet par octet
    """

    def __init__(self):
        self.state = State.IDLE
        self.length = 0
        self.buffer = bytearray()
        self.frames = []  # Trames complètes décodées
        self.errors = []  # Erreurs détectées

    def feed(self, data: bytes):
        """
        Alimente la machine à états avec des octets reçus

        Args:
            data: Octets reçus du port série
        """
        for byte in data:
            self._process_byte(byte)

    def _process_byte(self, byte: int):
        """Traite un octet selon l'état courant"""

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
        """Valide le CRC et extrait les données"""
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
        """Enregistre une erreur"""
        self.errors.append(message)

    def get_frame(self) -> Optional[dict]:
        """
        Récupère une trame décodée (FIFO)

        Returns:
            Dict avec les données ou None si pas de trame
        """
        if self.frames:
            return self.frames.pop(0)
        return None

    def get_error(self) -> Optional[str]:
        """
        Récupère une erreur (FIFO)

        Returns:
            Message d'erreur ou None
        """
        if self.errors:
            return self.errors.pop(0)
        return None


class UARTReceiver:
    """
    Récepteur UART pour données capteurs depuis le Pico

    Protocole: STX/ETX avec CRC-8, 32 bytes par trame
    Port: /dev/ttyAMA3 (UART3 sur GPIO8/9)
    """

    def __init__(self, port: str = '/dev/ttyAMA3', baudrate: int = 115200):
        """
        Initialise le récepteur UART

        Args:
            port: Port série (défaut: /dev/ttyAMA3 pour Pi5 UART3)
            baudrate: Vitesse (défaut: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self._running = False
        self._thread = None
        self._data_queue = Queue(maxsize=10)
        self._callback = None
        self._reader = FrameReader()

        # Statistiques
        self._packets_received = 0
        self._packets_lost = 0
        self._errors = 0
        self._last_sequence = -1

        # Dernières données reçues
        self._last_data = None
        self._lock = threading.Lock()

    def open(self) -> bool:
        """
        Ouvre la connexion série

        Returns:
            bool: True si réussi
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
        """Ferme la connexion série"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None

    def start_async(self, callback: Optional[Callable[[SensorData], None]] = None):
        """
        Démarre la réception en mode asynchrone (thread)

        Args:
            callback: Fonction appelée pour chaque paquet reçu
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
        """Boucle de réception (thread)"""
        while self._running:
            try:
                # Lire les octets disponibles
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self._reader.feed(data)

                # Traiter les trames décodées
                while True:
                    frame = self._reader.get_frame()
                    if frame is None:
                        break

                    # Détecter les trames perdues
                    self._check_sequence(frame['seq'])
                    self._packets_received += 1

                    # Convertir en SensorData
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
                        timestamp=time.time()
                    )

                    # Stocker dernières données
                    with self._lock:
                        self._last_data = sensor_data

                    # Callback ou queue
                    if self._callback:
                        self._callback(sensor_data)
                    else:
                        try:
                            self._data_queue.put_nowait(sensor_data)
                        except:
                            pass  # Queue pleine

                # Traiter les erreurs
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
        """Vérifie le compteur de séquence pour détecter les pertes"""
        if self._last_sequence >= 0:
            expected = (self._last_sequence + 1) % 65536
            if current_seq != expected:
                lost = (current_seq - expected) % 65536
                self._packets_lost += lost
        self._last_sequence = current_seq

    def read_sensor_data(self, timeout: float = 1.0) -> Optional[SensorData]:
        """
        Lit les données capteurs

        Args:
            timeout: Timeout en secondes

        Returns:
            SensorData ou None
        """
        if self._running:
            try:
                return self._data_queue.get(timeout=timeout)
            except Empty:
                return None
        return None

    def get_last_data(self) -> Optional[SensorData]:
        """Retourne les dernières données reçues (thread-safe)"""
        with self._lock:
            return self._last_data

    def get_stats(self) -> dict:
        """Retourne les statistiques de communication"""
        total = self._packets_received + self._packets_lost
        loss_rate = (self._packets_lost / total * 100) if total > 0 else 0

        return {
            'packets_received': self._packets_received,
            'packets_lost': self._packets_lost,
            'errors': self._errors,
            'loss_rate': f"{loss_rate:.2f}%"
        }


# Test du module
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
                    print(f"  GPS:   {data.latitude:.6f}deg, {data.longitude:.6f}deg")
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

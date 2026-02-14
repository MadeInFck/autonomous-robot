"""Wrapper autour du SDK rplidarc1 pour accumuler des scans complets."""

import asyncio
import logging
import time
from queue import Queue, Empty
from threading import Thread
from typing import Callable, Optional

from rplidarc1 import RPLidar

from lidar.data import Scan, ScanPoint

logger = logging.getLogger(__name__)


class LidarScanner:
    """Encapsule le SDK rplidarc1 et accumule les points en scans 360deg complets.

    Le SDK envoie les points un par un dans une asyncio.Queue.
    Cette classe :
      - Lance un thread asyncio dedie pour lire le LiDAR
      - Accumule les points et detecte les tours complets (angle qui repasse par ~0deg)
      - Expose les scans complets via une queue thread-safe ou un callback
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 460800,
        min_quality: int = 0,
        on_scan: Optional[Callable[[Scan], None]] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.min_quality = min_quality
        self.on_scan = on_scan

        self._lidar: Optional[RPLidar] = None
        self._scan_queue: Queue[Scan] = Queue()
        self._py_queue: Queue[dict] = Queue()
        self._thread: Optional[Thread] = None
        self._running = False

        # Accumulation du scan en cours
        self._current_points: list[ScanPoint] = []
        self._last_angle: float = -1.0

        # Dernier scan complet (pour l'API web)
        self._last_scan: Optional[Scan] = None

    def start(self):
        """Demarre la connexion et le scan."""
        logger.info(f"Connexion au LiDAR sur {self.port} @ {self.baudrate}")
        self._lidar = RPLidar(self.port, self.baudrate)
        self._running = True

        loop = asyncio.new_event_loop()
        self._thread = Thread(target=self._run_loop, args=(loop,), daemon=True)
        self._thread.start()
        logger.info("Thread asyncio demarre, scan en cours.")

    def stop(self):
        """Arrete proprement le scan et la connexion."""
        if not self._running:
            return
        self._running = False
        if self._lidar:
            self._lidar.stop_event.set()
        if self._thread:
            self._thread.join(timeout=3)
        if self._lidar:
            try:
                self._lidar.reset()
                self._lidar.shutdown()
            except Exception as e:
                logger.warning(f"Erreur lors de l'arret du LiDAR: {e}")
        logger.info("LiDAR arrete.")

    def get_scan(self, timeout: float = 1.0) -> Optional[Scan]:
        """Recupere le prochain scan complet (bloquant avec timeout)."""
        try:
            return self._scan_queue.get(timeout=timeout)
        except Empty:
            return None

    def get_last_scan(self) -> Optional[Scan]:
        """Retourne le dernier scan complet (non-bloquant, thread-safe)."""
        return self._last_scan

    def process_incoming(self):
        """Traite les points recus du thread asyncio et les accumule en scans.

        Appeler cette methode regulierement depuis le thread principal.
        """
        while True:
            try:
                point_dict = self._py_queue.get_nowait()
            except Empty:
                break
            self._accumulate_point(point_dict)

    def _accumulate_point(self, point_dict: dict):
        """Ajoute un point au scan en cours. Detecte les nouveaux tours."""
        point = ScanPoint.from_sdk_dict(point_dict)
        if point is None:
            return

        current_angle = point.angle_deg

        # Detection d'un nouveau tour : l'angle diminue de plus de 180deg
        if (
            self._last_angle >= 0
            and self._last_angle > 270
            and current_angle < 90
            and len(self._current_points) > 10
        ):
            scan = Scan(points=list(self._current_points), timestamp=time.time())
            self._last_scan = scan
            self._scan_queue.put(scan)
            if self.on_scan:
                self.on_scan(scan)
            logger.debug(
                f"Scan complet: {scan.num_points} points, "
                f"{scan.num_valid} valides, "
                f"qualite moyenne: {scan.mean_quality:.1f}"
            )
            self._current_points.clear()

        self._last_angle = current_angle

        if point.quality >= self.min_quality:
            self._current_points.append(point)

    # --- Thread asyncio interne ---

    def _run_loop(self, loop: asyncio.AbstractEventLoop):
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._async_runner())
        loop.close()

    async def _async_runner(self):
        task_scan = asyncio.create_task(
            self._lidar.simple_scan(make_return_dict=False)
        )
        task_forward = asyncio.create_task(self._forward_points())
        await self._lidar.stop_event.wait()
        task_scan.cancel()
        task_forward.cancel()

    async def _forward_points(self):
        """Transfere les points de l'asyncio.Queue vers la queue thread-safe."""
        while not self._lidar.stop_event.is_set():
            try:
                data = await asyncio.wait_for(
                    self._lidar.output_queue.get(), timeout=0.5
                )
                self._py_queue.put(data)
            except asyncio.TimeoutError:
                continue

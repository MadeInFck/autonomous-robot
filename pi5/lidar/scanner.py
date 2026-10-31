"""Wrapper around the rplidarc1 SDK to assemble complete scans."""

import asyncio
import logging
import time
from threading import Thread
from typing import Optional

from rplidarc1 import RPLidar

from lidar.data import Scan, ScanPoint

logger = logging.getLogger(__name__)


class LidarScanner:
    """Encapsulates the rplidarc1 SDK and assembles points into complete 360deg scans.

    The SDK sends points one by one into an asyncio.Queue. A dedicated thread
    runs the asyncio loop, folds each point into the current revolution as it
    arrives and publishes the last complete scan. Readers only ever see whole
    scans, they never have to pump anything, and nothing piles up when nobody
    reads.
    """

    def __init__(self, port: str, baudrate: int = 460800, min_quality: int = 0):
        self.port = port
        self.baudrate = baudrate
        self.min_quality = min_quality

        self._lidar: Optional[RPLidar] = None
        self._thread: Optional[Thread] = None
        self._running = False

        # Revolution being assembled — touched by the asyncio thread only
        self._current_points: list[ScanPoint] = []
        self._last_angle: float = -1.0

        # Last complete scan. Replaced wholesale (never mutated), so readers
        # in other threads need no lock.
        self._last_scan: Optional[Scan] = None

    def start(self):
        """Starts the connection and scanning."""
        logger.info(f"Connexion au LiDAR sur {self.port} @ {self.baudrate}")
        self._lidar = RPLidar(self.port, self.baudrate)
        self._running = True

        loop = asyncio.new_event_loop()
        self._thread = Thread(target=self._run_loop, args=(loop,), daemon=True)
        self._thread.start()
        logger.info("Thread asyncio demarre, scan en cours.")

    def stop(self):
        """Cleanly stops scanning and the connection."""
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

    def get_last_scan(self) -> Optional[Scan]:
        """Returns the last complete scan (non-blocking, thread-safe)."""
        return self._last_scan

    def _accumulate_point(self, point_dict: dict):
        """Adds a point to the current revolution, publishing it when it wraps."""
        point = ScanPoint.from_sdk_dict(point_dict)
        current_angle = point.angle_deg

        # New revolution: the angle wraps from the end of the turn to its start
        if (
            self._last_angle > 270
            and current_angle < 90
            and len(self._current_points) > 10
        ):
            self._last_scan = Scan(points=self._current_points, timestamp=time.time())
            self._current_points = []

        self._last_angle = current_angle

        if point.quality >= self.min_quality:
            self._current_points.append(point)

    # --- Internal asyncio thread ---

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
        """Folds the points coming out of the SDK queue into revolutions."""
        while not self._lidar.stop_event.is_set():
            try:
                data = await asyncio.wait_for(
                    self._lidar.output_queue.get(), timeout=0.5
                )
            except asyncio.TimeoutError:
                continue
            self._accumulate_point(data)

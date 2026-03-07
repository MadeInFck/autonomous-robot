"""
IMX500 AI Camera — EfficientDet Lite0 detection in a dedicated subprocess

The IMX500 inference runs on-chip. All Picamera2 calls run in a separate
OS process so they never contend with Flask or motor threads for the GIL.
Public API is unchanged from the threading version.
"""

import time
import threading
import multiprocessing as mp
from dataclasses import dataclass
from typing import List

try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore[assignment]

MODEL_PATH = '/usr/share/imx500-models/imx500_network_efficientdet_lite0_pp.rpk'
CONF_THRESHOLD = 0.50
MAX_HISTORY = 20
ALERT_COOLDOWN_S = 5.0  # same label won't re-trigger an alert within this window

# Full 80-class COCO label map
COCO_LABELS = {
    0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 4: "airplane",
    5: "bus", 6: "train", 7: "truck", 8: "boat", 9: "traffic light",
    10: "fire hydrant", 11: "stop sign", 12: "parking meter", 13: "bench",
    14: "bird", 15: "cat", 16: "dog", 17: "horse", 18: "sheep",
    19: "cow", 20: "elephant", 21: "bear", 22: "zebra", 23: "giraffe",
    24: "backpack", 25: "umbrella", 26: "handbag", 27: "tie",
    28: "suitcase", 29: "frisbee", 30: "skis", 31: "snowboard",
    32: "sports ball", 33: "kite", 34: "baseball bat", 35: "baseball glove",
    36: "skateboard", 37: "surfboard", 38: "tennis racket", 39: "bottle",
    40: "wine glass", 41: "cup", 42: "fork", 43: "knife", 44: "spoon",
    45: "bowl", 46: "banana", 47: "apple", 48: "sandwich", 49: "orange",
    50: "broccoli", 51: "carrot", 52: "hot dog", 53: "pizza",
    54: "donut", 55: "cake", 56: "chair", 57: "couch",
    58: "potted plant", 59: "bed", 60: "dining table", 61: "toilet",
    62: "tv", 63: "laptop", 64: "mouse", 65: "remote", 66: "keyboard",
    67: "cell phone", 68: "microwave", 69: "oven", 70: "toaster",
    71: "sink", 72: "refrigerator", 73: "book", 74: "clock",
    75: "vase", 76: "scissors", 77: "teddy bear", 78: "hair drier",
    79: "toothbrush",
}

# Labels that trigger alert notifications
# Note: 'bird' excluded — too many outdoor false positives
ALERT_LABELS = {
    'person',
    'cat', 'dog', 'horse', 'sheep', 'cow', 'bear', 'zebra', 'giraffe',
}


@dataclass
class Detection:
    label: str
    confidence: float    # 0.0 – 1.0
    timestamp: float     # time.time()
    is_alert: bool       # True = first occurrence within ALERT_COOLDOWN_S


# ---------------------------------------------------------------------------
# Camera subprocess entry point
# ---------------------------------------------------------------------------

def _camera_worker(model_path: str, conf_threshold: float,
                   result_q: mp.Queue, ctrl_q: mp.Queue):
    """Runs in a dedicated OS process — no GIL impact on the main process.

    ctrl_q commands (str): "resume", "pause", "stream_on", "stream_off", "stop"
    result_q messages (dict):
        {"t": "ready"}                         — subprocess initialised OK
        {"t": "error", "msg": str}             — fatal init error
        {"t": "frame", "det": [(label, conf)],
         "inference": bool, "jpeg": bytes}     — one captured frame
    """
    import time

    try:
        import cv2 as _cv2
    except ImportError:
        _cv2 = None

    try:
        from picamera2 import Picamera2
        from picamera2.devices import IMX500
        imx500 = IMX500(model_path)
        picam2 = Picamera2(imx500.camera_num)
        config = picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls={"FrameRate": 2},
            buffer_count=6,
        )
        picam2.configure(config)
    except Exception as e:
        result_q.put({"t": "error", "msg": str(e)})
        return

    result_q.put({"t": "ready"})

    paused = True
    streaming = False
    _fps_active = None  # last FrameRate actually set
    running = True

    while running:
        # Drain control queue (non-blocking)
        while True:
            try:
                cmd = ctrl_q.get_nowait()
            except Exception:
                break
            if cmd == "stop":
                running = False
            elif cmd == "resume":
                if paused:
                    try:
                        picam2.start()
                    except Exception:
                        pass
                paused = False
            elif cmd == "pause":
                if not paused:
                    try:
                        picam2.stop()
                    except Exception:
                        pass
                paused = True
                _fps_active = None
            elif cmd == "stream_on":
                streaming = True
            elif cmd == "stream_off":
                streaming = False

        if not running:
            break

        if paused:
            time.sleep(0.05)
            continue

        try:
            # Adjust FrameRate on streaming state change
            if streaming != _fps_active:
                picam2.set_controls({"FrameRate": 15 if streaming else 2})
                _fps_active = streaming

            frame = picam2.capture_array() if streaming else None
            metadata = picam2.capture_metadata()
            outputs = imx500.get_outputs(metadata)

            has_inference = outputs is not None
            detections = []
            jpeg = b''

            if has_inference:
                boxes = outputs[0]
                scores = outputs[1]
                classes = outputs[2]
                count = int(outputs[3][0])
                seen = set()
                raw = []
                for i in range(min(count, 100)):
                    conf = float(scores[i])
                    if conf < conf_threshold:
                        continue
                    label = COCO_LABELS.get(int(classes[i]))
                    if label is None or label in seen:
                        continue
                    raw.append((label, round(conf, 2), boxes[i]))
                    seen.add(label)

                detections = [(lb, cf) for lb, cf, _ in raw]

                if streaming and frame is not None and _cv2 is not None:
                    bgr = _cv2.cvtColor(frame, _cv2.COLOR_RGB2BGR)
                    h, w = bgr.shape[:2]
                    for label, conf, box in raw:
                        try:
                            y1, x1, y2, x2 = box
                            coords = imx500.convert_inference_coords(
                                (x1, y1, x2 - x1, y2 - y1), metadata, picam2)
                            rx, ry = int(coords.x), int(coords.y)
                            rw, rh = int(coords.width), int(coords.height)
                        except Exception:
                            y1, x1, y2, x2 = box
                            rx, ry = int(x1 * w / 320), int(y1 * h / 320)
                            rw, rh = int((x2 - x1) * w / 320), int((y2 - y1) * h / 320)
                        color = (0, 80, 255) if label in ALERT_LABELS else (0, 220, 80)
                        _cv2.rectangle(bgr, (rx, ry), (rx + rw, ry + rh), color, 2)
                        _cv2.putText(bgr, f"{label} {int(conf * 100)}%",
                                     (rx, max(ry - 6, 12)),
                                     _cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                    _, buf = _cv2.imencode('.jpg', bgr, [_cv2.IMWRITE_JPEG_QUALITY, 70])
                    jpeg = buf.tobytes()

            result_q.put({"t": "frame", "det": detections,
                          "inference": has_inference, "jpeg": jpeg})

        except Exception:
            pass  # silently skip errors between frames

    try:
        picam2.stop()
    except Exception:
        pass


# ---------------------------------------------------------------------------

class AiCamera:
    """Raspberry Pi AI Camera (IMX500) — public API unchanged.

    The camera pipeline runs in a dedicated subprocess so Picamera2's
    blocking calls never contend with Flask or motor threads for the GIL.

    Usage::

        cam = AiCamera()
        cam.start()          # spawns subprocess, starts paused
        cam.resume()         # starts picam2 inside subprocess
        detections = cam.get_recent_detections()
        cam.stop()
    """

    def __init__(self, conf_threshold: float = CONF_THRESHOLD):
        self._conf_threshold = conf_threshold
        self._lock = threading.Lock()
        self._history: List[Detection] = []
        self._latest_jpeg: bytes = b''
        self._stream_clients = 0
        self._running = False
        self._paused = True
        self._resuming = False
        self._error = False
        self._last_alert_time: dict = {}

        # Use spawn context: avoids fork+thread issues in the parent process
        self._ctx = mp.get_context('spawn')
        self._result_q: mp.Queue = self._ctx.Queue(maxsize=10)
        self._ctrl_q: mp.Queue = self._ctx.Queue(maxsize=10)
        self._process = None
        self._drain_thread = None

        # Availability check (non-Pi environments)
        try:
            import importlib.util
            if importlib.util.find_spec('picamera2') is None:
                raise ImportError("picamera2 not found")
        except ImportError as e:
            print(f"[Camera] Init error: {e}")
            self._error = True

    def start(self):
        """Spawn the camera subprocess. Starts paused — call resume() to activate."""
        if self._error:
            return
        self._running = True
        self._paused = True
        self._process = self._ctx.Process(
            target=_camera_worker,
            args=(MODEL_PATH, self._conf_threshold, self._result_q, self._ctrl_q),
            daemon=True,
        )
        self._process.start()
        # Wait for subprocess to confirm it initialised correctly
        try:
            msg = self._result_q.get(timeout=60)
            if msg.get("t") == "error":
                print(f"[Camera] Subprocess error: {msg.get('msg')}")
                self._error = True
                return
        except Exception:
            print("[Camera] Subprocess did not respond")
            self._error = True
            return
        # Drain thread: reads frames from subprocess, updates local state
        self._drain_thread = threading.Thread(target=self._drain_loop, daemon=True)
        self._drain_thread.start()
        print("[Camera] OK (paused — use resume() to activate)")

    def stop(self):
        self._running = False
        try:
            self._ctrl_q.put_nowait("stop")
        except Exception:
            pass
        if self._process:
            self._process.join(timeout=3)

    def pause(self):
        """Pause the camera pipeline (stops picam2 inside the subprocess)."""
        self._paused = True
        with self._lock:
            self._latest_jpeg = b''
            self._history = []
        try:
            self._ctrl_q.put_nowait("pause")
        except Exception:
            pass

    def resume(self):
        """Resume the camera pipeline. is_starting() is True until first inference."""
        self._resuming = True
        self._paused = False
        try:
            self._ctrl_q.put_nowait("resume")
        except Exception:
            self._resuming = False

    def is_paused(self) -> bool:
        return self._paused

    def is_starting(self) -> bool:
        """True between resume() and the first valid inference output (10-30 s)."""
        return self._resuming

    def _drain_loop(self):
        """Reads frames from the camera subprocess and updates local state."""
        _resume_start = None
        while self._running:
            try:
                msg = self._result_q.get(timeout=1.0)
            except Exception:
                continue

            if msg.get("t") != "frame":
                continue

            # Clear _resuming once the subprocess produces inference output
            if msg.get("inference"):
                if self._resuming:
                    self._resuming = False
                _resume_start = None
            elif self._resuming:
                # Timeout guard: abort _resuming after 60 s
                if _resume_start is None:
                    _resume_start = time.time()
                elif time.time() - _resume_start > 60:
                    print("[Camera] Timeout démarrage IMX500")
                    self._resuming = False
                    _resume_start = None

            jpeg = msg.get("jpeg", b'')
            detections = msg.get("det", [])

            with self._lock:
                self._latest_jpeg = jpeg
                now = time.time()
                new_detections = []
                for label, conf in detections:
                    last = self._last_alert_time.get(label, 0)
                    is_alert = (label in ALERT_LABELS) and (now - last >= ALERT_COOLDOWN_S)
                    if is_alert:
                        self._last_alert_time[label] = now
                    new_detections.append(Detection(
                        label=label, confidence=conf,
                        timestamp=now, is_alert=is_alert,
                    ))
                self._history = new_detections + self._history
                if len(self._history) > MAX_HISTORY:
                    self._history = self._history[:MAX_HISTORY]

    def get_recent_detections(self, max_age_s: float = 60.0) -> List[dict]:
        """Return detections newer than max_age_s, most recent first."""
        now = time.time()
        with self._lock:
            return [
                {
                    'label': d.label,
                    'confidence': d.confidence,
                    'timestamp': d.timestamp,
                    'age_s': round(now - d.timestamp, 1),
                    'is_alert': d.is_alert,
                }
                for d in self._history
                if now - d.timestamp <= max_age_s
            ]

    def get_latest_jpeg(self) -> bytes:
        """Return the most recent JPEG-encoded camera frame (bytes)."""
        with self._lock:
            return self._latest_jpeg

    def generate_stream(self):
        """MJPEG generator for Flask streaming response."""
        self._stream_clients += 1
        try:
            self._ctrl_q.put_nowait("stream_on")
        except Exception:
            pass
        try:
            while True:
                with self._lock:
                    frame = self._latest_jpeg
                if frame:
                    yield (
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                    )
                time.sleep(0.05)
        finally:
            self._stream_clients -= 1
            try:
                self._ctrl_q.put_nowait("stream_off")
            except Exception:
                pass

    def has_error(self) -> bool:
        return self._error

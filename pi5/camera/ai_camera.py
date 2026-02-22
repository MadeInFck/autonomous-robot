"""
IMX500 AI Camera — EfficientDet Lite0 background detection thread

Runs inference on-chip via the Sony IMX500 sensor.
Detection results are polled from the web server via /api/detections.
"""

import time
import threading
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


class AiCamera:
    """Background detection thread using the Raspberry Pi AI Camera (IMX500).

    Usage::

        cam = AiCamera()
        cam.start()
        # ... later ...
        detections = cam.get_recent_detections()
        cam.stop()
    """

    def __init__(self, conf_threshold: float = CONF_THRESHOLD):
        self._conf_threshold = conf_threshold
        self._lock = threading.Lock()
        self._history: List[Detection] = []   # most recent first, capped at MAX_HISTORY
        self._latest_jpeg: bytes = b''        # last encoded JPEG frame for streaming
        self._running = False
        self._paused = False
        self._thread = None
        self._error = False
        self._last_alert_time: dict = {}       # label -> timestamp of last alert

        try:
            from picamera2 import Picamera2
            from picamera2.devices import IMX500
            self._imx500 = IMX500(MODEL_PATH)
            self._picam2 = Picamera2(self._imx500.camera_num)
            config = self._picam2.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"},
                controls={"FrameRate": 15},
                buffer_count=6,
            )
            self._picam2.configure(config)
        except Exception as e:
            print(f"[Camera] Init error: {e}")
            self._error = True

    def start(self):
        if self._error:
            return
        self._picam2.start()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[Camera] OK")

    def stop(self):
        self._running = False
        if not self._error and hasattr(self, '_picam2'):
            try:
                self._picam2.stop()
            except Exception:
                pass

    def pause(self):
        """Pause detection and streaming.
        Drops frame rate to 1fps to reduce IMX500 NNA load (~93% reduction)
        without stopping the pipeline (stop/start breaks inference — model
        upload on start() takes 10-30s and get_outputs() returns None until done)."""
        self._paused = True
        with self._lock:
            self._latest_jpeg = b''
            self._history = []
        try:
            self._picam2.set_controls({"FrameRate": 1})
        except Exception:
            pass

    def resume(self):
        """Resume detection and streaming at full frame rate."""
        try:
            self._picam2.set_controls({"FrameRate": 15})
        except Exception:
            pass
        self._paused = False

    def is_paused(self) -> bool:
        return self._paused

    def _loop(self):
        while self._running:
            if self._paused:
                time.sleep(0.1)
                continue
            try:
                frame = self._picam2.capture_array()
                metadata = self._picam2.capture_metadata()
                outputs = self._imx500.get_outputs(metadata)

                raw = self._parse_raw(outputs) if outputs is not None else []

                # Encode frame to JPEG, drawing bounding boxes if detections exist
                if cv2 is not None:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    h_frame, w_frame = frame_bgr.shape[:2]
                    for label, conf, box in raw:
                        try:
                            y1, x1, y2, x2 = box
                            coords = self._imx500.convert_inference_coords(
                                (x1, y1, x2 - x1, y2 - y1), metadata, self._picam2)
                            rx, ry = int(coords.x), int(coords.y)
                            rw, rh = int(coords.width), int(coords.height)
                        except Exception:
                            # Fallback: scale from model input (320×320) to frame dims
                            y1, x1, y2, x2 = box
                            rx = int(x1 * w_frame / 320)
                            ry = int(y1 * h_frame / 320)
                            rw = int((x2 - x1) * w_frame / 320)
                            rh = int((y2 - y1) * h_frame / 320)
                        color = (0, 80, 255) if label in ALERT_LABELS else (0, 220, 80)
                        cv2.rectangle(frame_bgr, (rx, ry), (rx + rw, ry + rh), color, 2)
                        cv2.putText(frame_bgr, f"{label} {int(conf * 100)}%",
                                    (rx, max(ry - 6, 12)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                    _, buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    jpeg_bytes = buf.tobytes()
                else:
                    jpeg_bytes = b''

                with self._lock:
                    self._latest_jpeg = jpeg_bytes
                    now = time.time()
                    new_detections = []
                    for label, conf, _ in raw:
                        last = self._last_alert_time.get(label, 0)
                        is_alert = (label in ALERT_LABELS) and (now - last >= ALERT_COOLDOWN_S)
                        if is_alert:
                            self._last_alert_time[label] = now
                        new_detections.append(Detection(
                            label=label,
                            confidence=conf,
                            timestamp=now,
                            is_alert=is_alert,
                        ))
                    # Prepend to history, keep capped
                    self._history = new_detections + self._history
                    if len(self._history) > MAX_HISTORY:
                        self._history = self._history[:MAX_HISTORY]

            except Exception:
                pass   # silently skip errors between frames

    def _parse_raw(self, outputs) -> List[tuple]:
        """Extract (label, confidence) pairs from one frame's inference outputs."""
        boxes   = outputs[0]
        scores  = outputs[1]
        classes = outputs[2]
        count   = int(outputs[3][0])

        result = []
        seen_labels = set()
        for i in range(min(count, 100)):
            conf = float(scores[i])
            if conf < self._conf_threshold:
                continue
            label = COCO_LABELS.get(int(classes[i]))
            if label is None:
                continue
            # Keep only the highest-confidence detection per label per frame
            if label not in seen_labels:
                result.append((label, round(conf, 2)))
                seen_labels.add(label)

        return result

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
        while True:
            with self._lock:
                frame = self._latest_jpeg
            if frame:
                yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                )
            time.sleep(0.05)   # ~20 fps max

    def has_error(self) -> bool:
        return self._error

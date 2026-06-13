# Pip NumPy 2.x in ~/.local breaks ROS apt packages (cv_bridge, cv2, matplotlib).
# Load system NumPy 1.x first; re-add ~/.local later only for ultralytics.
import sys
import types

sys.path = [p for p in sys.path if ".local" not in p]

import os
from dataclasses import dataclass

def default_model_path() -> str:
    vexu_home = os.environ.get("VEXU_HOME", os.path.expanduser("~/VEXU_GHOST"))
    return os.path.join(vexu_home, "11_Robots", "push_back_cv", "models", "best.pt")

def _enable_ultralytics_imports() -> None:
    """Ultralytics pulls matplotlib at import time; inference does not need it."""
    if "matplotlib" not in sys.modules:
        matplotlib = types.ModuleType("matplotlib")
        matplotlib.pyplot = types.ModuleType("matplotlib.pyplot")
        sys.modules["matplotlib"] = matplotlib
        sys.modules["matplotlib.pyplot"] = matplotlib.pyplot

    import site

    user_site = site.getusersitepackages()
    if user_site not in sys.path:
        sys.path.insert(0, user_site)

import numpy as np
from sensor_msgs.msg import Image


def image_msg_to_bgr8(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to HxWx3 BGR uint8 without cv_bridge."""
    row_bytes = msg.step
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, row_bytes)
    img = img[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        img = img[:, :, ::-1].copy()
    elif msg.encoding != "bgr8":
        raise ValueError(f"Unsupported color encoding: {msg.encoding}")
    return img

def image_msg_to_depth_u16(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to HxW uint16 depth (mm) without cv_bridge."""
    if msg.encoding != "16UC1":
        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")
    row_vals = msg.step // np.dtype(np.uint16).itemsize
    img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, row_vals)
    return img[:, : msg.width]

def median_depth_m(depth_image: np.ndarray, cx: int, cy: int) -> float | None:
    """Read depth in a small window around the ball center (millimeters -> meters)."""
    h, w = depth_image.shape
    x0, x1 = max(cx - 2, 0), min(cx + 3, w)
    y0, y1 = max(cy - 2, 0), min(cy + 3, h)
    roi = depth_image[y0:y1, x0:x1]
    valid = roi[roi > 0].flatten()
    if valid.size < 3:
        return None
    return float(np.median(valid)) / 1000.0


def pixel_to_xyz(cx: int, cy: int, depth_m: float, fx: float, fy: float, cx0: float, cy0: float):
    """
    Pinhole camera math: pixel + depth -> 3D point in camera optical frame.

    Optical frame (RealSense / ROS convention):
      X = right, Y = down, Z = forward (into the scene)
    """
    x = (cx - cx0) * depth_m / fx
    y = (cy - cy0) * depth_m / fy
    z = depth_m
    return x, y, z

@dataclass
class RawDetection:
    cx: int
    cy: int
    depth_m: float
    class_name: str
    confidence: float

    @property
    def is_red(self) -> bool:
        return self.class_name == "red"

class YoloDetector:
    def __init__(self, model_path: str, device: str, conf_threshold: float):
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"Model not found: {model_path}")
        _enable_ultralytics_imports()
        from ultralytics import YOLO
        self.conf_threshold = conf_threshold
        self.device = device
        self.model = YOLO(model_path, task="detect")
        warmup = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model.predict(warmup, device=device, verbose=False)
    def detect(self, bgr_image: np.ndarray, depth_image: np.ndarray) -> list[RawDetection]:
        results = self.model.predict(
            bgr_image,
            device=self.device,
            conf=self.conf_threshold,
            verbose=False,
        )
        detections: list[RawDetection] = []
        for result in results:
            if result.boxes is None or len(result.boxes) == 0:
                continue
            for box in result.boxes:
                class_name = result.names[int(box.cls)]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                depth_m = median_depth_m(depth_image, cx, cy)
                if depth_m is None:
                    continue
                detections.append(RawDetection(
                    cx=cx, cy=cy, depth_m=depth_m,
                    class_name=class_name, confidence=float(box.conf),
                ))
        return detections

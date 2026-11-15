"""Pluggable detector backends.

Every backend is a callable object: `backend.infer(bgr_image) -> list of detection dicts`
with keys class_name, confidence and a box as either center format (x, y, width, height)
or corner format (x1, y1, x2, y2), all in pixels of the image that was passed in.

To test YOUR model, either use one of the built-in backends or write a function
`def detect(bgr): ...` and point the `custom_callable` parameter at it ("pkg.module:detect").
"""
import base64
import importlib
import json
import os
import urllib.request

import numpy as np

DEFAULT_ROBOFLOW_MODEL = "ras-yolo/vex-override-w4dvt-4-yolo26m-t1"
DEFAULT_KEY_FILE = "/etc/ghost/roboflow_api_key"


def normalize_detection(p):
    """Accept Roboflow / ultralytics-ish dicts and return the canonical form."""
    d = {"class_name": str(p.get("class_name") or p.get("class") or p.get("name") or "object"),
         "confidence": float(p.get("confidence", 1.0))}
    if "x1" in p:
        d.update(x1=float(p["x1"]), y1=float(p["y1"]), x2=float(p["x2"]), y2=float(p["y2"]))
    else:
        d.update(x=float(p["x"]), y=float(p["y"]), width=float(p["width"]), height=float(p["height"]))
    return d


class RoboflowBackend:
    """Same hosted workflow call the Jetson `cv_camera_node` makes (serverless Roboflow).

    The API key is read from $ROBOFLOW_API_KEY, else from `api_key_file`. It is never logged.
    """

    def __init__(self, model_id=DEFAULT_ROBOFLOW_MODEL, api_url="https://serverless.roboflow.com",
                 api_key_file=DEFAULT_KEY_FILE, timeout=20.0, jpeg_quality=90, **_):
        key = os.environ.get("ROBOFLOW_API_KEY", "").strip()
        if not key and os.path.isfile(api_key_file):
            with open(api_key_file, encoding="utf-8") as f:
                key = f.read().strip()
        if not key:
            raise RuntimeError(
                "No Roboflow API key. Run `export ROBOFLOW_API_KEY=...` in the shell you launch "
                "from (for Docker: put it in your host env before `docker compose up`), or use "
                "backend:=color_blob / ultralytics / custom.")
        self._key = key
        self._url = api_url.rstrip("/") + "/workflows/run"
        self._timeout = timeout
        self._quality = int(jpeg_quality)
        self._spec = {
            "version": "1.0",
            "inputs": [{"type": "InferenceImage", "name": "image"}],
            "steps": [{"type": "roboflow_core/roboflow_object_detection_model@v2",
                       "name": "model", "images": "$inputs.image", "model_id": model_id}],
            "outputs": [{"type": "JsonField", "name": "predictions",
                         "coordinates_system": "own", "selector": "$steps.model.predictions"}],
        }

    def infer(self, bgr):
        import cv2
        ok, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if not ok:
            raise RuntimeError("could not JPEG-encode frame")
        body = json.dumps({
            "specification": self._spec,
            "images": {"image": {"type": "base64", "value": base64.b64encode(enc.tobytes()).decode("ascii")}},
        }).encode("utf-8")
        req = urllib.request.Request(self._url, data=body, method="POST", headers={
            "Content-Type": "application/json", "Authorization": "Bearer " + self._key})
        with urllib.request.urlopen(req, timeout=self._timeout) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
        out = payload["outputs"][0]["predictions"]
        preds = out.get("predictions") if isinstance(out, dict) else out
        if preds is None:
            raise RuntimeError("Roboflow response had no prediction data")
        return [normalize_detection(p) for p in preds]


class UltralyticsBackend:
    """Local YOLO weights (.pt / .onnx) through the `ultralytics` package (pip install it)."""

    def __init__(self, weights="", conf=0.25, imgsz=640, device="cpu", **_):
        if not weights:
            raise RuntimeError("backend:=ultralytics needs the `weights` parameter (path to .pt/.onnx)")
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise RuntimeError("pip3 install ultralytics (inside the container) to use this backend") from e
        self._model = YOLO(weights)
        self._kw = dict(conf=float(conf), imgsz=int(imgsz), device=device, verbose=False)

    def infer(self, bgr):
        res = self._model.predict(bgr, **self._kw)[0]
        names = res.names
        out = []
        for b in res.boxes:
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            out.append({"class_name": names[int(b.cls[0])], "confidence": float(b.conf[0]),
                        "x1": x1, "y1": y1, "x2": x2, "y2": y2})
        return out


class CustomBackend:
    """Import `package.module:function` and call it with the BGR frame."""

    def __init__(self, custom_callable="", **_):
        if ":" not in custom_callable:
            raise RuntimeError("custom_callable must look like 'package.module:function'")
        mod, fn = custom_callable.split(":", 1)
        self._fn = getattr(importlib.import_module(mod), fn)

    def infer(self, bgr):
        return [normalize_detection(p) for p in self._fn(bgr)]


class ColorBlobBackend:
    """Trivial HSV-threshold detector: red / blue / yellow blobs.

    NOT a real model. It exists so the whole sim -> detect -> score loop can be checked
    end to end with no network, key or weights (use evaluator `class_agnostic:=true` with it).
    """

    RANGES = {"red": [((0, 120, 80), (8, 255, 255)), ((172, 120, 80), (180, 255, 255))],
              "blue": [((100, 120, 60), (130, 255, 255))],
              "yellow": [((20, 120, 100), (35, 255, 255))]}

    def __init__(self, min_area=80, **_):
        self._min_area = int(min_area)

    def infer(self, bgr):
        import cv2
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        out = []
        for name, ranges in self.RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) < self._min_area:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                out.append({"class_name": name, "confidence": 0.5,
                            "x1": float(x), "y1": float(y), "x2": float(x + w), "y2": float(y + h)})
        return out


BACKENDS = {"roboflow": RoboflowBackend, "ultralytics": UltralyticsBackend,
            "custom": CustomBackend, "color_blob": ColorBlobBackend}


def make_backend(name, **kwargs):
    if name not in BACKENDS:
        raise RuntimeError("unknown backend '%s' (choose from %s)" % (name, sorted(BACKENDS)))
    return BACKENDS[name](**kwargs)

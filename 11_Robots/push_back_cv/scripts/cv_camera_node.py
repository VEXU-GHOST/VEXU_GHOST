#!/usr/bin/env python3
"""Run the trained VEX detector on the camera plugged into this Jetson.

Frames are captured from a V4L2 camera (default /dev/video0) as fast as the
camera delivers them. Match videos are written to ~/match_videos when
recording is started from the live page or by publishing true on /cv/record.
Watch the live annotated image in a browser at http://<jetson-ip>:8080.

The HTTP workflow spec is inlined so this stays on the system Python that ROS
uses. Installing inference-sdk would pull NumPy 2 and break ROS packages.
"""

import sys

# Pip NumPy 2.x in ~/.local breaks ROS apt packages. Prefer system packages.
sys.path = [p for p in sys.path if ".local" not in p]

import base64
import ctypes
import errno
import fcntl
import mmap
import os
import queue
import select
import shutil
import socket
import struct
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
    import numpy as np
    import requests
except ModuleNotFoundError as error:
    raise SystemExit(
        f"Missing dependency: {error.name}.\n"
        "On the Jetson, OpenCV ships with JetPack (python3 -c \"import cv2\").\n"
        "Install requests with: sudo apt-get install -y python3-requests"
    ) from None

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ghost_msgs.msg import V5SensorUpdate

# Roboflow object-detection model. Versionless id uses the deployed version.
MODEL_ID = "ras-yolo/vex-override-w4dvt-4-yolo26m-t1"
API_URL = "https://serverless.roboflow.com"
API_KEY_FILE = "/etc/ghost/roboflow_api_key"

WORKFLOW = {
    "version": "1.0",
    "inputs": [{"type": "InferenceImage", "name": "image"}],
    "steps": [
        {
            "type": "roboflow_core/roboflow_object_detection_model@v2",
            "name": "model",
            "images": "$inputs.image",
            "model_id": MODEL_ID,
        }
    ],
    "outputs": [
        {
            "type": "JsonField",
            "name": "predictions",
            "coordinates_system": "own",
            "selector": "$steps.model.predictions",
        }
    ],
}


def load_api_key() -> str:
    key = os.environ.get("ROBOFLOW_API_KEY", "").strip()
    if not key and os.path.isfile(API_KEY_FILE):
        with open(API_KEY_FILE, encoding="utf-8") as handle:
            key = handle.read().strip()
    if not key:
        raise RuntimeError(
            "No Roboflow API key. Export ROBOFLOW_API_KEY, or on the Jetson run:\n"
            f"  sudo mkdir -p /etc/ghost && sudo tee {API_KEY_FILE} >/dev/null"
        )
    return key


def infer_frame(frame, api_key: str, api_url: str, transport=requests):
    success, encoded = cv2.imencode(".jpg", frame)
    if not success:
        raise RuntimeError("Could not encode the camera frame.")
    response = transport.post(
        f"{api_url}/workflows/run",
        headers={"Authorization": f"Bearer {api_key}"},
        json={
            "specification": WORKFLOW,
            "inputs": {
                "image": {
                    "type": "base64",
                    "value": base64.b64encode(encoded).decode("ascii"),
                }
            },
            "excluded_fields": ["image"],
            "disable_sinks": True,
        },
        timeout=(5, 30),
    )
    response.raise_for_status()
    outputs = response.json().get("outputs", [])
    if not outputs or not isinstance(outputs[0].get("predictions"), dict):
        raise RuntimeError("The model did not return prediction data.")
    return outputs[0]["predictions"]


def _box_color(class_name: str):
    name = class_name.lower()
    if "red" in name:
        return (0, 0, 255)
    if "blue" in name:
        return (255, 80, 0)
    return (0, 220, 0)


def annotate_frame(frame, data, scale: float = 1.0):
    if not data:
        return frame
    annotated = frame.copy()
    for prediction in data.get("predictions", []):
        if not isinstance(prediction, dict):
            continue
        class_name = str(prediction.get("class", "object"))
        confidence = float(prediction.get("confidence", 0.0))
        if not all(key in prediction for key in ("x", "y", "width", "height")):
            continue
        x1 = int((prediction["x"] - prediction["width"] / 2) * scale)
        y1 = int((prediction["y"] - prediction["height"] / 2) * scale)
        x2 = int((prediction["x"] + prediction["width"] / 2) * scale)
        y2 = int((prediction["y"] + prediction["height"] / 2) * scale)
        color = _box_color(class_name)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            annotated,
            f"{class_name} {confidence:.0%}",
            (x1, max(y1 - 8, 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
    return annotated


# H.264 target size. 4 Mb/s is about 60 MB for a two-minute match.
RECORD_BITRATE = 4_000_000


def gst_module():
    """GStreamer, if the Jetson image has the Python bindings."""
    try:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        return Gst
    except Exception:
        return None


def jpeg_size(data: bytes):
    """Width and height from a JPEG header, without decoding the image."""
    if len(data) < 4 or data[0:2] != b"\xff\xd8":
        return None
    index = 2
    while index + 9 < len(data):
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        if marker in (0xC0, 0xC1, 0xC2):
            height, width = struct.unpack_from(">HH", data, index + 5)
            return int(width), int(height)
        if marker in (0xD8, 0xD9):
            index += 2
            continue
        segment = struct.unpack_from(">H", data, index + 2)[0]
        index += 2 + segment
    return None


def _v4l2_ioc(direction, number, size):
    """Build a V4L2 ioctl request. Direction is 1 write, 2 read, 3 read/write."""
    value = (direction << 30) | (size << 16) | (ord("V") << 8) | number
    return ctypes.c_int32(value).value


def _xioctl(fd, request, payload):
    raw = bytearray(ctypes.string_at(ctypes.addressof(payload), ctypes.sizeof(payload)))
    while True:
        try:
            fcntl.ioctl(fd, request, raw, True)
            break
        except OSError as error:
            if error.errno == errno.EINTR:
                continue
            raise
    ctypes.memmove(ctypes.addressof(payload), bytes(raw), ctypes.sizeof(payload))


class _V4L2Fract(ctypes.Structure):
    _fields_ = [("numerator", ctypes.c_uint32), ("denominator", ctypes.c_uint32)]


class _V4L2PixFormat(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("pixelformat", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("bytesperline", ctypes.c_uint32),
        ("sizeimage", ctypes.c_uint32),
        ("colorspace", ctypes.c_uint32),
        ("priv", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("ycbcr_enc", ctypes.c_uint32),
        ("quantization", ctypes.c_uint32),
        ("xfer_func", ctypes.c_uint32),
    ]


class _V4L2Format(ctypes.Structure):
    class _Union(ctypes.Union):
        # The real C union contains a pointer, so it is 8-byte aligned.
        # That puts the image format at offset 8 and makes the struct 208 bytes.
        _fields_ = [
            ("pix", _V4L2PixFormat),
            ("raw", ctypes.c_uint8 * 200),
            ("_align", ctypes.c_void_p),
        ]

    _fields_ = [("type", ctypes.c_uint32), ("fmt", _Union)]


class _V4L2FmtDesc(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("description", ctypes.c_char * 32),
        ("pixelformat", ctypes.c_uint32),
        ("mbus_code", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]


class _V4L2FrmSizeDiscrete(ctypes.Structure):
    _fields_ = [("width", ctypes.c_uint32), ("height", ctypes.c_uint32)]


class _V4L2FrmSizeStepwise(ctypes.Structure):
    _fields_ = [
        ("min_width", ctypes.c_uint32),
        ("max_width", ctypes.c_uint32),
        ("step_width", ctypes.c_uint32),
        ("min_height", ctypes.c_uint32),
        ("max_height", ctypes.c_uint32),
        ("step_height", ctypes.c_uint32),
    ]


class _V4L2FrmSizeEnum(ctypes.Structure):
    class _Union(ctypes.Union):
        _fields_ = [
            ("discrete", _V4L2FrmSizeDiscrete),
            ("stepwise", _V4L2FrmSizeStepwise),
        ]

    _fields_ = [
        ("index", ctypes.c_uint32),
        ("pixel_format", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("size", _Union),
        ("reserved", ctypes.c_uint32 * 2),
    ]


class _V4L2FrmIvalStepwise(ctypes.Structure):
    _fields_ = [
        ("min", _V4L2Fract),
        ("max", _V4L2Fract),
        ("step", _V4L2Fract),
    ]


class _V4L2FrmIvalEnum(ctypes.Structure):
    class _Union(ctypes.Union):
        _fields_ = [("discrete", _V4L2Fract), ("stepwise", _V4L2FrmIvalStepwise)]

    _fields_ = [
        ("index", ctypes.c_uint32),
        ("pixel_format", ctypes.c_uint32),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("ival", _Union),
        ("reserved", ctypes.c_uint32 * 2),
    ]


class _V4L2CaptureParm(ctypes.Structure):
    _fields_ = [
        ("capability", ctypes.c_uint32),
        ("capturemode", ctypes.c_uint32),
        ("timeperframe", _V4L2Fract),
        ("extendedmode", ctypes.c_uint32),
        ("readbuffers", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 4),
    ]


class _V4L2StreamParm(ctypes.Structure):
    class _Union(ctypes.Union):
        _fields_ = [("capture", _V4L2CaptureParm), ("raw", ctypes.c_uint8 * 200)]

    _fields_ = [("type", ctypes.c_uint32), ("parm", _Union)]


class _V4L2RequestBuffers(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("flags", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class _V4L2Timeval(ctypes.Structure):
    _fields_ = [("sec", ctypes.c_long), ("usec", ctypes.c_long)]


class _V4L2Timecode(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("frames", ctypes.c_uint8),
        ("seconds", ctypes.c_uint8),
        ("minutes", ctypes.c_uint8),
        ("hours", ctypes.c_uint8),
        ("userbits", ctypes.c_uint8 * 4),
    ]


class _V4L2Buffer(ctypes.Structure):
    class _Memory(ctypes.Union):
        _fields_ = [
            ("offset", ctypes.c_uint32),
            ("userptr", ctypes.c_ulong),
            ("fd", ctypes.c_int32),
        ]

    class _Request(ctypes.Union):
        _fields_ = [("request_fd", ctypes.c_int32), ("reserved", ctypes.c_uint32)]

    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("bytesused", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("field", ctypes.c_uint32),
        ("timestamp", _V4L2Timeval),
        ("timecode", _V4L2Timecode),
        ("sequence", ctypes.c_uint32),
        ("memory", ctypes.c_uint32),
        ("m", _Memory),
        ("length", ctypes.c_uint32),
        ("reserved2", ctypes.c_uint32),
        ("request", _Request),
    ]


class _V4L2Control(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("value", ctypes.c_int32)]


_V4L2_BUF_TYPE = 1
_V4L2_MEMORY_MMAP = 1
_V4L2_FIELD_NONE = 1
_V4L2_FRMSIZE_DISCRETE = 1
_V4L2_FRMSIZE_STEPWISE = 3
_V4L2_FRMIVAL_DISCRETE = 1
_V4L2_FRMIVAL_STEPWISE = 3
_V4L2_PIX_FMT_MJPG = 0x47504A4D
_V4L2_CID_EXPOSURE_AUTO_PRIORITY = 0x009A0903
_V4L2_CID_EXPOSURE_AUTO = 0x009A0901
_V4L2_EXPOSURE_APERTURE_PRIORITY = 3

_VIDIOC_ENUM_FMT = _v4l2_ioc(3, 2, ctypes.sizeof(_V4L2FmtDesc))
_VIDIOC_S_FMT = _v4l2_ioc(3, 5, ctypes.sizeof(_V4L2Format))
_VIDIOC_G_FMT = _v4l2_ioc(3, 4, ctypes.sizeof(_V4L2Format))
_VIDIOC_REQBUFS = _v4l2_ioc(3, 8, ctypes.sizeof(_V4L2RequestBuffers))
_VIDIOC_QUERYBUF = _v4l2_ioc(3, 9, ctypes.sizeof(_V4L2Buffer))
_VIDIOC_QBUF = _v4l2_ioc(3, 15, ctypes.sizeof(_V4L2Buffer))
_VIDIOC_DQBUF = _v4l2_ioc(3, 17, ctypes.sizeof(_V4L2Buffer))
_VIDIOC_STREAMON = _v4l2_ioc(1, 18, ctypes.sizeof(ctypes.c_uint32))
_VIDIOC_STREAMOFF = _v4l2_ioc(1, 19, ctypes.sizeof(ctypes.c_uint32))
_VIDIOC_G_PARM = _v4l2_ioc(3, 21, ctypes.sizeof(_V4L2StreamParm))
_VIDIOC_S_PARM = _v4l2_ioc(3, 22, ctypes.sizeof(_V4L2StreamParm))
_VIDIOC_S_CTRL = _v4l2_ioc(3, 28, ctypes.sizeof(_V4L2Control))
_VIDIOC_ENUM_FRAMESIZES = _v4l2_ioc(3, 74, ctypes.sizeof(_V4L2FrmSizeEnum))
_VIDIOC_ENUM_FRAMEINTERVALS = _v4l2_ioc(3, 75, ctypes.sizeof(_V4L2FrmIvalEnum))


def _interval_fps(numerator, denominator):
    if numerator <= 0 or denominator <= 0:
        return 0.0
    return float(denominator) / float(numerator)


class MjpegCamera:
    """Reads the camera's own JPEG frames at the rate the sensor is set to.

    OpenCV's capture path decodes every frame before it returns. With one
    queued buffer, that decode holds the camera at about half of 90 fps.
    """

    def __init__(self, device_index):
        self.path = f"/dev/video{int(device_index)}"
        self.fd = None
        self.maps = []
        self.streaming = False
        self.driver_fps = 0.0
        self.fps_locked = False
        self.dropped = 0
        self._last_sequence = None
        self.width = 0
        self.height = 0

    def isOpened(self):
        return self.fd is not None and self.streaming

    def open(self):
        self.fd = os.open(self.path, os.O_RDWR | os.O_NONBLOCK)

    def close(self):
        self.stop()
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def release(self):
        self.close()

    def list_modes(self):
        modes = []
        seen = set()
        for width, height in self._sizes():
            for fps in self._rates(width, height):
                mode = (width, height, int(round(fps)))
                if mode[2] <= 0 or mode in seen:
                    continue
                seen.add(mode)
                modes.append(mode)
        return modes

    def _sizes(self):
        sizes = []
        index = 0
        while index < 64:
            desc = _V4L2FrmSizeEnum()
            desc.index = index
            desc.pixel_format = _V4L2_PIX_FMT_MJPG
            try:
                _xioctl(self.fd, _VIDIOC_ENUM_FRAMESIZES, desc)
            except OSError:
                break
            if desc.type == _V4L2_FRMSIZE_DISCRETE:
                sizes.append((int(desc.size.discrete.width), int(desc.size.discrete.height)))
            else:
                sizes.extend(self._step_sizes(desc.size.stepwise))
                break
            index += 1
        return sizes

    def _step_sizes(self, stepwise):
        wanted = [(1920, 1080), (1280, 720), (960, 540), (640, 480), (640, 360), (320, 240)]
        found = []
        min_w, max_w = int(stepwise.min_width), int(stepwise.max_width)
        min_h, max_h = int(stepwise.min_height), int(stepwise.max_height)
        step_w = max(int(stepwise.step_width), 1)
        step_h = max(int(stepwise.step_height), 1)
        candidates = wanted + [(max_w, max_h)]
        for width, height in candidates:
            if not (min_w <= width <= max_w and min_h <= height <= max_h):
                continue
            width -= (width - min_w) % step_w
            height -= (height - min_h) % step_h
            if (width, height) not in found:
                found.append((width, height))
        return found

    def _rates(self, width, height):
        rates = []
        index = 0
        while index < 64:
            desc = _V4L2FrmIvalEnum()
            desc.index = index
            desc.pixel_format = _V4L2_PIX_FMT_MJPG
            desc.width = width
            desc.height = height
            try:
                _xioctl(self.fd, _VIDIOC_ENUM_FRAMEINTERVALS, desc)
            except OSError:
                break
            if desc.type == _V4L2_FRMIVAL_DISCRETE:
                rates.append(
                    _interval_fps(desc.ival.discrete.numerator, desc.ival.discrete.denominator)
                )
            else:
                fastest = _interval_fps(desc.ival.stepwise.min.numerator, desc.ival.stepwise.min.denominator)
                slowest = _interval_fps(desc.ival.stepwise.max.numerator, desc.ival.stepwise.max.denominator)
                if fastest:
                    rates.append(fastest)
                if slowest and fastest and slowest <= 60 <= fastest:
                    rates.append(60.0)
                break
            index += 1
        return rates

    def _set_ctrl(self, control_id, value):
        ctrl = _V4L2Control()
        ctrl.id = control_id
        ctrl.value = value
        try:
            _xioctl(self.fd, _VIDIOC_S_CTRL, ctrl)
            return True
        except OSError:
            return False

    def lock_fps(self):
        """Restore auto-exposure for normal brightness while keeping frame rate fixed."""
        self._set_ctrl(_V4L2_CID_EXPOSURE_AUTO, _V4L2_EXPOSURE_APERTURE_PRIORITY)
        self.fps_locked = self._set_ctrl(_V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0)
        return self.fps_locked

    def start(self, width, height, fps):
        self.stop()
        fmt = _V4L2Format()
        fmt.type = _V4L2_BUF_TYPE
        fmt.fmt.pix.width = int(width)
        fmt.fmt.pix.height = int(height)
        fmt.fmt.pix.pixelformat = _V4L2_PIX_FMT_MJPG
        fmt.fmt.pix.field = _V4L2_FIELD_NONE
        _xioctl(self.fd, _VIDIOC_S_FMT, fmt)
        _xioctl(self.fd, _VIDIOC_G_FMT, fmt)
        if fmt.fmt.pix.pixelformat != _V4L2_PIX_FMT_MJPG:
            raise OSError(errno.EINVAL, "camera did not accept MJPEG")
        self.width = int(fmt.fmt.pix.width)
        self.height = int(fmt.fmt.pix.height)
        self.lock_fps()
        parm = _V4L2StreamParm()
        parm.type = _V4L2_BUF_TYPE
        parm.parm.capture.timeperframe.numerator = 1
        parm.parm.capture.timeperframe.denominator = int(round(fps))
        try:
            _xioctl(self.fd, _VIDIOC_S_PARM, parm)
        except OSError:
            pass
        try:
            _xioctl(self.fd, _VIDIOC_G_PARM, parm)
            self.driver_fps = _interval_fps(
                parm.parm.capture.timeperframe.numerator,
                parm.parm.capture.timeperframe.denominator,
            )
        except OSError:
            self.driver_fps = float(fps)
        request = _V4L2RequestBuffers()
        request.count = 16
        request.type = _V4L2_BUF_TYPE
        request.memory = _V4L2_MEMORY_MMAP
        _xioctl(self.fd, _VIDIOC_REQBUFS, request)
        if request.count < 2:
            raise OSError(errno.ENOMEM, "camera did not allocate frame buffers")
        self.maps = []
        for index in range(int(request.count)):
            buf = _V4L2Buffer()
            buf.type = _V4L2_BUF_TYPE
            buf.memory = _V4L2_MEMORY_MMAP
            buf.index = index
            _xioctl(self.fd, _VIDIOC_QUERYBUF, buf)
            mapped = mmap.mmap(
                self.fd,
                int(buf.length),
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE,
                offset=int(buf.m.offset),
            )
            self.maps.append(mapped)
            _xioctl(self.fd, _VIDIOC_QBUF, buf)
        kind = ctypes.c_uint32(_V4L2_BUF_TYPE)
        _xioctl(self.fd, _VIDIOC_STREAMON, kind)
        self.streaming = True
        self._last_sequence = None
        self.dropped = 0
        # Auto exposure can drop a 90 fps camera to 45 once streaming starts.
        self.lock_fps()
        try:
            _xioctl(self.fd, _VIDIOC_S_PARM, parm)
            _xioctl(self.fd, _VIDIOC_G_PARM, parm)
            self.driver_fps = _interval_fps(
                parm.parm.capture.timeperframe.numerator,
                parm.parm.capture.timeperframe.denominator,
            )
        except OSError:
            pass
        return self.width, self.height, self.driver_fps

    def stop(self):
        if self.fd is None:
            return
        if self.streaming:
            kind = ctypes.c_uint32(_V4L2_BUF_TYPE)
            try:
                _xioctl(self.fd, _VIDIOC_STREAMOFF, kind)
            except OSError:
                pass
            self.streaming = False
        for mapped in self.maps:
            mapped.close()
        self.maps = []
        if self.fd is None:
            return
        request = _V4L2RequestBuffers()
        request.count = 0
        request.type = _V4L2_BUF_TYPE
        request.memory = _V4L2_MEMORY_MMAP
        try:
            _xioctl(self.fd, _VIDIOC_REQBUFS, request)
        except OSError:
            pass

    def read_jpeg(self, timeout=1.0):
        if not self.streaming:
            return None
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        buf = _V4L2Buffer()
        buf.type = _V4L2_BUF_TYPE
        buf.memory = _V4L2_MEMORY_MMAP
        try:
            _xioctl(self.fd, _VIDIOC_DQBUF, buf)
        except OSError as error:
            if error.errno == errno.EAGAIN:
                return None
            raise
        try:
            size = int(buf.bytesused)
            packet = bytes(memoryview(self.maps[buf.index])[:size]) if size else b""
            if self._last_sequence is not None and buf.sequence != self._last_sequence + 1:
                gap = int(buf.sequence) - int(self._last_sequence) - 1
                if gap > 0:
                    self.dropped += gap
            self._last_sequence = int(buf.sequence)
        finally:
            try:
                _xioctl(self.fd, _VIDIOC_QBUF, buf)
            except OSError:
                pass
        start = packet.find(b"\xff\xd8", 0, 32)
        if start > 0:
            packet = packet[start:]
        if not packet.startswith(b"\xff\xd8"):
            return None
        return packet

    def measure(self, seconds):
        warmup = time.monotonic() + 0.2
        while time.monotonic() < warmup:
            self.read_jpeg(timeout=0.2)
        started = time.monotonic()
        count = 0
        while time.monotonic() - started < seconds:
            if self.read_jpeg(timeout=0.5):
                count += 1
        return count / max(time.monotonic() - started, 0.001)


class MjpegAviWriter:
    """Writes the camera's original JPEG frames. No second compression step."""

    def __init__(self, path: str, width: int, height: int, fps: float):
        self.path = path
        self.width = width
        self.height = height
        self.fps = max(1.0, float(fps))
        self.frame_count = 0
        self.dropped_frames = 0
        self._t0 = None
        self._t_last = None
        self._queue = queue.SimpleQueue()
        self._stop_event = threading.Event()
        self._handle = open(path, "wb", buffering=4 * 1024 * 1024)
        self._index = []
        self._write_header()
        self._movi_data = self._handle.tell()
        self._worker = threading.Thread(target=self._writer_loop, daemon=True)
        self._worker.start()

    def _write_header(self):
        handle = self._handle
        fps = self.fps
        handle.write(b"RIFF")
        self._riff_size_at = handle.tell()
        handle.write(struct.pack("<I", 0))
        handle.write(b"AVI ")
        handle.write(b"LIST")
        hdrl_size_at = handle.tell()
        handle.write(struct.pack("<I", 0))
        hdrl_start = handle.tell()
        handle.write(b"hdrl")
        avih = struct.pack(
            "<14I",
            int(1_000_000 / fps),
            0,
            0,
            0x10,
            0,
            0,
            1,
            0,
            self.width,
            self.height,
            0,
            0,
            0,
            0,
        )
        handle.write(b"avih")
        handle.write(struct.pack("<I", len(avih)))
        self._avih_data_at = handle.tell()
        handle.write(avih)
        handle.write(b"LIST")
        strl_size_at = handle.tell()
        handle.write(struct.pack("<I", 0))
        strl_start = handle.tell()
        handle.write(b"strl")
        strh = struct.pack(
            "<4s4sIHHIIIIIIIIhhhh",
            b"vids",
            b"MJPG",
            0,
            0,
            0,
            0,
            1,
            int(round(fps)),
            0,
            0,
            0,
            0xFFFFFFFF,
            0,
            0,
            0,
            self.width,
            self.height,
        )
        handle.write(b"strh")
        handle.write(struct.pack("<I", len(strh)))
        self._strh_data_at = handle.tell()
        handle.write(strh)
        strf = struct.pack(
            "<IiiHHIIiiII",
            40,
            self.width,
            self.height,
            1,
            24,
            0x47504A4D,
            self.width * self.height * 3,
            0,
            0,
            0,
            0,
        )
        handle.write(b"strf")
        handle.write(struct.pack("<I", len(strf)))
        handle.write(strf)
        strl_end = handle.tell()
        handle.seek(strl_size_at)
        handle.write(struct.pack("<I", strl_end - strl_start))
        handle.seek(strl_end)
        hdrl_end = handle.tell()
        handle.seek(hdrl_size_at)
        handle.write(struct.pack("<I", hdrl_end - hdrl_start))
        handle.seek(hdrl_end)
        handle.write(b"LIST")
        self._movi_size_at = handle.tell()
        handle.write(struct.pack("<I", 0))
        handle.write(b"movi")

    def _emit(self, jpeg: bytes):
        """Append one unique camera JPEG."""
        if len(jpeg) % 2:
            jpeg += b"\x00"
        offset = self._handle.tell() - (self._movi_data - 4)
        self._handle.write(b"00dc")
        self._handle.write(struct.pack("<I", len(jpeg)))
        self._handle.write(jpeg)
        self._index.append((offset, len(jpeg)))
        self.frame_count += 1

    def write(self, jpeg: bytes, when: float | None = None):
        if self._stop_event.is_set():
            return
        now = time.monotonic() if when is None else when
        if self._t0 is None:
            self._t0 = now
        self._t_last = now
        self._queue.put(jpeg)

    def _writer_loop(self):
        while True:
            try:
                jpeg = self._queue.get(timeout=0.05)
            except queue.Empty:
                if self._stop_event.is_set():
                    break
                continue
            if jpeg is None:
                break
            self._emit(jpeg)

    def _patch_timing(self, frames: int, elapsed: float):
        if frames < 1 or elapsed <= 0:
            return
        scale = 1000
        rate = max(1, int(round(frames * scale / elapsed)))
        usec = max(1, int(round(elapsed * 1_000_000 / frames)))
        end = self._handle.tell()
        self._handle.seek(self._avih_data_at)
        self._handle.write(struct.pack("<I", usec))
        self._handle.seek(self._avih_data_at + 16)
        self._handle.write(struct.pack("<I", frames))
        self._handle.seek(self._strh_data_at + 20)
        self._handle.write(struct.pack("<II", scale, rate))
        self._handle.seek(self._strh_data_at + 32)
        self._handle.write(struct.pack("<I", frames))
        self._handle.seek(end)

    def close(self, elapsed: float | None = None):
        if self._stop_event.is_set() and self._handle.closed:
            return
        self._stop_event.set()
        self._queue.put(None)
        self._worker.join(timeout=15.0)
        while not self._queue.empty():
            try:
                jpeg = self._queue.get_nowait()
            except queue.Empty:
                break
            if jpeg is not None:
                self._emit(jpeg)
        if self._handle.closed:
            return
        if elapsed is None:
            if self._t0 is not None and self._t_last is not None and self._t_last > self._t0:
                elapsed = self._t_last - self._t0
            elif self._t0 is not None:
                elapsed = time.monotonic() - self._t0
        if elapsed and elapsed > 0 and self.frame_count >= 1:
            self._patch_timing(self.frame_count, elapsed)
        self._finish()

    def _finish(self):
        if self._handle.closed:
            return
        index_start = self._handle.tell()
        movi_end = index_start
        self._handle.write(b"idx1")
        self._handle.write(struct.pack("<I", len(self._index) * 16))
        for offset, size in self._index:
            self._handle.write(struct.pack("<4sIII", b"00dc", 0x10, offset, size))
        end = self._handle.tell()
        self._handle.seek(self._riff_size_at)
        self._handle.write(struct.pack("<I", end - 8))
        self._handle.seek(self._movi_size_at)
        self._handle.write(struct.pack("<I", movi_end - self._movi_size_at - 4))
        self._handle.close()


def restamp_avi(path: str, frame_count: int, elapsed_sec: float) -> bool:
    """Make an AVI play for the real recording length, not the encoder's guess.

    OpenCV labels the file with a fixed frame rate when it is created. If the
    encoder then saves fewer frames than that rate, playback runs fast. The
    AVI header is rewritten so frame_count frames last elapsed_sec seconds.
    """
    if frame_count < 1 or elapsed_sec <= 0:
        return False
    usec = max(1, int(round(elapsed_sec * 1_000_000 / frame_count)))
    scale = 1000
    rate = max(1, int(round(frame_count * scale / elapsed_sec)))
    with open(path, "r+b") as handle:
        # Only the header is needed. Reading a multi-gigabyte match here was
        # failing on the Jetson and leaving the file stamped at 90 fps.
        blob = handle.read(65536)
        avih = blob.find(b"avih")
        if avih < 0:
            return False
        base = avih + 8
        handle.seek(base)
        handle.write(struct.pack("<I", usec))
        handle.seek(base + 16)
        handle.write(struct.pack("<I", frame_count))
        pos = 0
        while True:
            strh = blob.find(b"strh", pos)
            if strh < 0:
                break
            data = strh + 8
            if blob[data:data + 4] == b"vids":
                handle.seek(data + 20)
                handle.write(struct.pack("<II", scale, rate))
                handle.seek(data + 32)
                handle.write(struct.pack("<I", frame_count))
            pos = strh + 4
    return True


def put_latest(destination, value):
    try:
        destination.put_nowait(value)
    except queue.Full:
        try:
            destination.get_nowait()
        except queue.Empty:
            pass
        destination.put_nowait(value)


class LatestJpeg:
    """One annotated JPEG, replaced in place. Browser clients wait for a new one."""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self._closed = False

    def publish(self, jpeg: bytes):
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def wait(self, last_seq: int):
        with self._cond:
            while self._seq == last_seq and not self._closed:
                self._cond.wait(timeout=1.0)
            if self._closed or self._jpeg is None:
                return None, last_seq
            return self._jpeg, self._seq

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()


class _LiveHandler(BaseHTTPRequestHandler):
    # HTTP/1.0 keeps the multipart stream open without a Content-Length.
    protocol_version = "HTTP/1.0"

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/stream":
            self._stream()
            return
        if self.path.split("?", 1)[0] == "/record/status":
            body = self.server.node.record_status().encode()
            self._send(200, "text/plain; charset=utf-8", body)
            return
        if self.path.split("?", 1)[0] == "/record/start":
            body = self.server.node.start_recording().encode()
            self._send(200, "text/plain; charset=utf-8", body)
            return
        if self.path.split("?", 1)[0] == "/record/stop":
            body = self.server.node.stop_recording().encode()
            self._send(200, "text/plain; charset=utf-8", body)
            return
        body = (
            "<!doctype html><title>VEX camera</title>"
            "<body style=\"margin:0;background:#111;color:#eee;font-family:sans-serif\">"
            "<p style=\"padding:12px 12px 0\">"
            "<button onclick=\"act('/record/start')\">Start recording</button> "
            "<button onclick=\"act('/record/stop')\">Stop recording</button> "
            "<span id=\"state\"></span></p>"
            "<img src=\"/stream\" style=\"width:100%;height:auto\">"
            "<script>"
            "function act(path){fetch(path).then(function(r){return r.text()}).then(function(t){document.getElementById('state').textContent=t}).catch(function(){document.getElementById('state').textContent='Button request failed'})}"
            "function refresh(){fetch('/record/status').then(function(r){return r.text()})"
            ".then(function(t){document.getElementById('state').textContent=t})}"
            "refresh();setInterval(refresh,1000);"
            "</script></body>"
        ).encode()
        self._send(200, "text/html; charset=utf-8", body)

    def _send(self, code, content_type, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self):
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        seq = 0
        try:
            while True:
                jpeg, seq = self.server.latest.wait(seq)
                if jpeg is None:
                    break
                chunk = (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return


def start_live_server(port: int, latest: LatestJpeg):
    try:
        server = ThreadingHTTPServer(("0.0.0.0", port), _LiveHandler)
    except OSError as error:
        return None, error
    server.latest = latest
    server.node = None
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, None


def _bgr_image_msg(frame, stamp) -> Image:
    image = np.ascontiguousarray(frame)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = "camera_link"
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(image.shape[1] * 3)
    msg.data = image.tobytes()
    return msg


class CvCameraNode(Node):
    def __init__(self):
        super().__init__("cv_camera")
        self.declare_parameter("camera_device", 0)
        self.declare_parameter("frame_width", 1920)
        self.declare_parameter("frame_height", 1080)
        self.declare_parameter("frame_fps", 60)
        self.declare_parameter("show_window", False)
        self.declare_parameter("live_port", 8080)
        self.declare_parameter("video_dir", os.path.expanduser("~/match_videos"))
        # Detection is off by default. Videos still record without the model.
        self.declare_parameter("run_model", False)
        # Record automatically while the V5 reports autonomous or driver control.
        self.declare_parameter("auto_record", True)
        # Disabled gaps shorter than this (auton -> driver) stay in one video.
        self.declare_parameter("match_end_grace_sec", 5.0)

        self.run_model = bool(self.get_parameter("run_model").value)
        self.auto_record = bool(self.get_parameter("auto_record").value)
        self.match_end_grace_sec = float(self.get_parameter("match_end_grace_sec").value)
        self.api_key = load_api_key() if self.run_model else ""
        self.api_url = API_URL
        self.camera_device = int(self.get_parameter("camera_device").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.frame_fps = int(self.get_parameter("frame_fps").value)
        self.video_dir = os.path.expanduser(str(self.get_parameter("video_dir").value))
        self.show_window = bool(self.get_parameter("show_window").value) and bool(
            os.environ.get("DISPLAY")
        )

        self.image_pub = self.create_publisher(Image, "/cv/annotated_image", 10)
        self.label_pub = self.create_publisher(String, "/cv/detection_labels", 10)
        self.create_subscription(Bool, "/cv/record", self._on_record_cmd, 10)
        self._match_active = False
        self._was_disabled = True
        self._disabled_since = None
        if self.auto_record:
            self.create_subscription(
                V5SensorUpdate,
                "/v5/sensor_update",
                self._on_sensor_update,
                qos_profile_sensor_data,
            )
            self.create_timer(0.5, self._auto_record_tick)
        self.frames = queue.Queue(maxsize=1)
        self.results = queue.Queue(maxsize=1)
        self.stopped = threading.Event()
        self.camera = None
        self.latest_data = None
        self.status = "Connecting to model..."
        self.latest_jpeg = LatestJpeg()
        self._latest_lock = threading.Lock()
        self._latest_frame = None
        self._latest_jpeg = None
        self._jpeg_writer = None
        self._frame_times = deque(maxlen=120)
        self.capture_fps = 0.0
        self._record_lock = threading.Lock()
        self._writer = None
        self._record_path = None
        self._record_frames = 0
        self._record_t0 = 0.0
        self._record_fps = 30.0
        self._next_write = 0.0
        self._record_cmd = None
        self._record_error = None
        self._pipeline = None
        self._appsink = None
        self._gst = gst_module()
        self._use_gst = self._gst is not None
        self._mode = (self.frame_width, self.frame_height, self.frame_fps)
        self._driver_fps = 0.0
        self._read_misses = 0
        self.live_port = int(self.get_parameter("live_port").value)
        self.live_server, live_error = start_live_server(self.live_port, self.latest_jpeg)
        if self.live_server is not None:
            self.live_server.node = self

        self.worker = None
        if self.run_model:
            self.worker = threading.Thread(target=self._infer_latest_frames, daemon=True)
            self.worker.start()
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        self.create_timer(1.0 / 15.0, self._on_preview)
        if self.live_server is None:
            self.get_logger().error(f"Live view port {self.live_port} failed: {live_error}")
        else:
            host_ip = "<jetson-ip>"
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.connect(("8.8.8.8", 80))
                    host_ip = sock.getsockname()[0]
            except OSError:
                pass
            self.get_logger().info(
                f"Live view: http://{host_ip}:{self.live_port}  "
                f"Videos: {self.video_dir}"
            )
        model_note = f"model {MODEL_ID}" if self.run_model else "model off"
        self.get_logger().info(
            f"VEX camera ({model_note}, auto record {'on' if self.auto_record else 'off'}) "
            f"on device {self.camera_device}. {self._disk_summary()}"
        )

    def _note_camera_frame(self):
        now = time.monotonic()
        self._frame_times.append(now)
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            if span > 0:
                self.capture_fps = (len(self._frame_times) - 1) / span

    def _on_cam_buffer(self, pad, info):
        self._note_camera_frame()
        return self._gst.PadProbeReturn.OK

    def _mode_candidates(self):
        modes = []
        sizes = (
            (self.frame_width, self.frame_height),
            (1920, 1080),
            (1280, 720),
            (640, 480),
        )
        for size in sizes:
            for fps in (self.frame_fps, 60, 30):
                mode = (size[0], size[1], fps)
                if mode not in modes:
                    modes.append(mode)
        return modes

    def _gst_pipeline_description(self, width, height, fps, record_path):
        device = f"/dev/video{self.camera_device}"
        # Preview queue drops frames so a slow browser never slows the camera.
        # The record branch keeps every frame and stores the camera timestamps,
        # so playback stays at real-time speed.
        record = ""
        if record_path:
            record = (
                " t. ! queue max-size-buffers=60 max-size-bytes=0 max-size-time=0 "
                "! jpegparse ! nvv4l2decoder mjpeg=1 "
                "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
                f"! nvv4l2h264enc bitrate={RECORD_BITRATE} control-rate=1 "
                f"maxperf-enable=1 insert-sps-pps=1 iframeinterval={fps} "
                "! h264parse ! qtmux ! "
                f'filesink sync=false location="{record_path}"'
            )
        return (
            f"v4l2src name=cam device={device} do-timestamp=true "
            f"! image/jpeg,width={width},height={height},framerate={fps}/1 "
            "! tee name=t "
            " t. ! queue max-size-buffers=1 leaky=downstream "
            "! jpegparse ! jpegdec ! videoconvert ! video/x-raw,format=BGR "
            "! appsink name=preview max-buffers=1 drop=true sync=false "
            f"{record}"
        )

    def _stop_pipeline(self, send_eos):
        pipeline = self._pipeline
        self._pipeline = None
        self._appsink = None
        if pipeline is None:
            return
        if send_eos:
            pipeline.send_event(self._gst.Event.new_eos())
            pipeline.get_bus().timed_pop_filtered(
                8 * self._gst.SECOND,
                self._gst.MessageType.EOS | self._gst.MessageType.ERROR,
            )
        pipeline.set_state(self._gst.State.NULL)

    def _start_pipeline(self, width, height, fps, record_path) -> str | None:
        self._stop_pipeline(send_eos=False)
        self._frame_times.clear()
        description = self._gst_pipeline_description(width, height, fps, record_path)
        try:
            pipeline = self._gst.parse_launch(description)
        except Exception as error:
            return str(error)
        cam = pipeline.get_by_name("cam")
        pad = cam.get_static_pad("src") if cam is not None else None
        if pad is not None:
            pad.add_probe(self._gst.PadProbeType.BUFFER, self._on_cam_buffer)
        pipeline.set_state(self._gst.State.PLAYING)
        change, _state, _pending = pipeline.get_state(3 * self._gst.SECOND)
        if change == self._gst.StateChangeReturn.FAILURE:
            message = pipeline.get_bus().pop_filtered(self._gst.MessageType.ERROR)
            detail = ""
            if message is not None:
                err, debug = message.parse_error()
                detail = f"{err.message} {debug}"
            pipeline.set_state(self._gst.State.NULL)
            return detail or "camera pipeline did not start"
        self._pipeline = pipeline
        self._appsink = pipeline.get_by_name("preview")
        self._mode = (width, height, fps)
        return None

    def _choose_gst_mode(self) -> bool:
        best = None
        for width, height, fps in self._mode_candidates():
            error = self._start_pipeline(width, height, fps, None)
            if error:
                self.get_logger().warning(
                    f"Camera {width}x{height}@{fps} unavailable: {error}",
                    throttle_duration_sec=2.0,
                )
                continue
            time.sleep(1.0)
            measured = self.capture_fps
            self.get_logger().info(
                f"Camera mode {width}x{height} requested {fps} fps, measured {measured:.0f} fps."
            )
            score = (measured, width * height)
            if best is None or score > best[0]:
                best = (score, (width, height, fps), measured)
            if measured >= fps * 0.85:
                self.get_logger().info(
                    f"Using {width}x{height} at {measured:.0f} fps."
                )
                return True
            self._stop_pipeline(send_eos=False)
        if best is None:
            return False
        width, height, fps = best[1]
        error = self._start_pipeline(width, height, fps, None)
        if error:
            return False
        self.get_logger().info(
            f"Using {width}x{height}. Camera delivered {best[2]:.0f} fps."
        )
        return True

    def _ranked_modes(self, modes):
        target = self.frame_fps
        preferred_sizes = {(1920, 1080): 3, (1280, 720): 2, (1280, 960): 1}
        matching = [m for m in modes if abs(m[2] - target) <= 5]
        if matching:
            return sorted(
                matching,
                key=lambda m: (preferred_sizes.get((m[0], m[1]), 0), m[0] * m[1], m[2]),
                reverse=True,
            )
        under = [m for m in modes if m[2] <= target] or modes
        return sorted(
            under,
            key=lambda m: (m[2], preferred_sizes.get((m[0], m[1]), 0), m[0] * m[1]),
            reverse=True,
        )

    def _accept_camera(self, camera, width, height, fps, driver_fps, measured):
        camera.dropped = 0
        camera._last_sequence = None
        self.camera = camera
        self._mode = (width, height, int(round(driver_fps or fps)))
        self._driver_fps = driver_fps
        self._frame_times.clear()
        self.capture_fps = measured
        self.get_logger().info(
            f"Opened {camera.path} at {width}x{height}, delivered {measured:.0f} fps "
            f"(driver {driver_fps:.0f})."
        )

    def _open_camera(self) -> bool:
        if self.camera is not None and self.camera.isOpened():
            return True
        camera = MjpegCamera(self.camera_device)
        try:
            camera.open()
        except OSError as error:
            self.get_logger().error(
                f"Could not open {camera.path}: {error}. "
                "Plug the camera into the Jetson and confirm this user is in the video group "
                "(sudo usermod -aG video $USER, then log in again).",
                throttle_duration_sec=5.0,
            )
            return False
        try:
            modes = camera.list_modes()
        except OSError as error:
            modes = []
            self.get_logger().warning(f"Could not list camera modes: {error}")
        if modes:
            self.get_logger().info(
                "Camera MJPEG modes: " + ", ".join(f"{w}x{h}@{fps}" for w, h, fps in modes)
            )
        else:
            modes = self._mode_candidates()
            self.get_logger().warning(
                f"Camera did not list MJPEG modes. Trying {self.frame_fps} fps at each resolution."
            )
        best = None
        for width, height, fps in self._ranked_modes(modes):
            try:
                actual_w, actual_h, driver_fps = camera.start(width, height, fps)
            except OSError as error:
                self.get_logger().warning(
                    f"Camera rejected {width}x{height}@{fps}: {error}",
                    throttle_duration_sec=2.0,
                )
                camera.stop()
                continue
            measured = camera.measure(0.8)
            self.get_logger().info(
                f"Tried {actual_w}x{actual_h} requested {fps} fps. "
                f"Driver {driver_fps:.0f} fps, delivered {measured:.0f} fps. "
                f"Frame-rate lock {camera.fps_locked}."
            )
            score = (measured, actual_w * actual_h)
            if best is None or score > best[0]:
                best = (score, actual_w, actual_h, fps, driver_fps, measured)
            if measured >= self.frame_fps * 0.85:
                self._accept_camera(camera, actual_w, actual_h, fps, driver_fps, measured)
                return True
            camera.stop()
        if best is None:
            camera.close()
            self.get_logger().error(
                "Camera opened but did not deliver a JPEG frame.",
                throttle_duration_sec=5.0,
            )
            return False
        _score, actual_w, actual_h, fps, driver_fps, measured = best
        try:
            actual_w, actual_h, driver_fps = camera.start(actual_w, actual_h, fps)
        except OSError as error:
            camera.close()
            self.get_logger().error(f"Could not start the best camera mode: {error}")
            return False
        self._accept_camera(camera, actual_w, actual_h, fps, driver_fps, measured)
        self.get_logger().warning(
            f"No camera mode delivered {self.frame_fps} fps. Using {actual_w}x{actual_h} at {measured:.0f} fps."
        )
        return True

    def _pull_gst_frame(self):
        sample = self._appsink.emit("try-pull-sample", 100 * self._gst.MSECOND)
        if sample is None:
            return None
        caps = sample.get_caps().get_structure(0)
        width = caps.get_value("width")
        height = caps.get_value("height")
        buf = sample.get_buffer()
        ok, info = buf.map(self._gst.MapFlags.READ)
        if not ok:
            return None
        try:
            frame = np.frombuffer(info.data, dtype=np.uint8).reshape(height, width, 3).copy()
        finally:
            buf.unmap(info)
        return frame

    def _apply_record_cmd(self, cmd):
        if self._use_gst and self._pipeline is not None:
            width, height, fps = self._mode
            if cmd == "":
                self._stop_pipeline(send_eos=True)
                error = self._start_pipeline(width, height, fps, None)
                with self._record_lock:
                    self._record_path = None
                    self._record_error = error
                return
            error = self._start_pipeline(width, height, fps, cmd)
            with self._record_lock:
                if error:
                    self._record_error = error
                    self._record_path = None
                    self._start_pipeline(width, height, fps, None)
                else:
                    self._record_path = cmd
                    self._record_frames = 0
                    self._record_t0 = time.monotonic()
                    self._record_error = None
            return
        if cmd == "":
            self._close_opencv_writer()
            return
        self._open_opencv_writer(cmd)

    def _file_fps(self) -> float:
        measured = self.capture_fps
        if measured >= 50:
            return 60.0
        if measured >= 24:
            return 30.0
        if measured >= 5:
            return float(round(measured))
        return float(self._mode[2])

    def _open_opencv_writer(self, path):
        with self._latest_lock:
            frame = self._latest_frame
            size = None if frame is None else (frame.shape[1], frame.shape[0])
        if size is None:
            self._record_error = "Camera is not ready yet."
            return
        fps = self._file_fps()
        gst = (
            "appsrc ! videoconvert ! video/x-raw,format=NV12 "
            "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2h264enc bitrate={RECORD_BITRATE} maxperf-enable=1 "
            f"insert-sps-pps=1 iframeinterval={int(fps)} "
            f'! h264parse ! qtmux ! filesink location="{path}"'
        )
        writer = cv2.VideoWriter(gst, cv2.CAP_GSTREAMER, 0, fps, size)
        if not writer.isOpened():
            writer.release()
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not writer.isOpened():
            self._record_error = f"Could not open {path}"
            return
        self._writer = writer
        self._record_path = path
        self._record_frames = 0
        self._record_t0 = time.monotonic()
        self._record_fps = fps
        self._next_write = time.monotonic()
        self._record_error = None

    def _close_opencv_writer(self):
        writer = self._writer
        self._writer = None
        self._record_path = None
        if writer is not None:
            writer.release()

    def _capture_loop(self):
        while not self.stopped.is_set():
            cmd = self._record_cmd
            if cmd is not None:
                self._record_cmd = None
                self._apply_record_cmd(cmd)
            if not self._open_camera():
                time.sleep(0.5)
                continue
            try:
                packet = self.camera.read_jpeg()
            except OSError as error:
                self.get_logger().error(
                    f"Camera read failed: {error}",
                    throttle_duration_sec=5.0,
                )
                self.camera.release()
                self.camera = None
                continue
            if not packet:
                self._read_misses += 1
                if self._read_misses >= 20:
                    self.get_logger().error(
                        "Camera stopped delivering frames. Reopening it.",
                        throttle_duration_sec=5.0,
                    )
                    self.camera.release()
                    self.camera = None
                    self._read_misses = 0
                continue
            self._read_misses = 0
            self._note_camera_frame()
            writer = self._jpeg_writer
            if writer is not None:
                writer.write(packet)
                self._record_frames += 1
            with self._latest_lock:
                self._latest_jpeg = packet
            dropped = self.camera.dropped
            if dropped and dropped != getattr(self, "_logged_drops", 0):
                self._logged_drops = dropped
                self.get_logger().warning(
                    f"Camera dropped {dropped} frames while this script was busy.",
                    throttle_duration_sec=5.0,
                )

    def _packet_jpeg(self, sample):
        raw = np.ascontiguousarray(sample).tobytes()
        if raw.startswith(b"\xff\xd8"):
            return raw
        if sample.ndim == 3 and sample.shape[2] == 3:
            ok, encoded = cv2.imencode(
                ".jpg", sample, [int(cv2.IMWRITE_JPEG_QUALITY), 100]
            )
            if ok:
                return encoded.tobytes()
        return None

    def _record_loop(self):
        """Save frames until Stop. Playback length is corrected when the file closes."""
        while not self._record_stop.is_set():
            with self._latest_lock:
                frame = None if self._latest_frame is None else self._latest_frame.copy()
            writer = self._writer
            if frame is None or writer is None:
                time.sleep(0.01)
                continue
            started = time.monotonic()
            writer.write(frame)
            self._record_frames += 1
            leftover = (1.0 / 60.0) - (time.monotonic() - started)
            if leftover > 0:
                time.sleep(leftover)

    def _disk_summary(self) -> str:
        folder = self.video_dir if os.path.isdir(self.video_dir) else os.path.dirname(self.video_dir) or "/"
        usage = shutil.disk_usage(folder if os.path.isdir(folder) else "/")
        free_gb = usage.free / (1024 ** 3)
        # 4 Mb/s H.264 is about 60 MB for a two-minute match.
        matches = int(usage.free / (60 * 1024 * 1024))
        return f"{free_gb:.1f} GB free, about {matches} two-minute matches."

    def record_status(self) -> str:
        with self._record_lock:
            path = self._record_path
            frames = self._record_frames
        fps = self.capture_fps
        summary = self._disk_summary()
        if not path:
            if self._record_error:
                return self._record_error
            return (
                f"Not recording. Camera {fps:.0f} fps, sensor {self._driver_fps:.0f} fps. "
                f"{summary} Folder: {self.video_dir}"
            )
        elapsed = max(time.monotonic() - self._record_t0, 0.001)
        saved = frames / elapsed
        return (
            f"Recording {os.path.basename(path)} saving {saved:.0f} fps. "
            f"Camera {fps:.0f} fps, sensor {self._driver_fps:.0f} fps. {summary}"
        )

    def _on_record_cmd(self, msg: Bool):
        if msg.data:
            self.get_logger().info(self.start_recording())
        else:
            self.get_logger().info(self.stop_recording())

    def _on_sensor_update(self, msg: V5SensorUpdate):
        if msg.competition_status.is_disabled:
            self._was_disabled = True
            if self._disabled_since is None:
                self._disabled_since = time.monotonic()
        else:
            self._disabled_since = None
            # Only arm on disabled -> enabled so a manual stop is not undone mid-match.
            if self._was_disabled:
                self._was_disabled = False
                self._match_active = True

    def _auto_record_tick(self):
        """Record from the start of auton/driver until the V5 stays disabled."""
        if self._match_active:
            if (
                self._disabled_since is not None
                and time.monotonic() - self._disabled_since >= self.match_end_grace_sec
            ):
                self._match_active = False
                self.get_logger().info(f"Match ended. {self.stop_recording()}")
                return
            if self._jpeg_writer is None:
                # Retries each tick if the camera was not ready at match start.
                self.start_recording()

    def start_recording(self) -> str:
        with self._record_lock:
            if self._jpeg_writer is not None:
                return f"Already recording {self._record_path}"
            with self._latest_lock:
                jpeg = self._latest_jpeg
            if not jpeg:
                self._record_error = "Camera is not ready yet."
                return self._record_error
            size = jpeg_size(jpeg)
            if size is None:
                width, height = self._mode[0], self._mode[1]
            else:
                width, height = size
            os.makedirs(self.video_dir, exist_ok=True)
            path = os.path.join(self.video_dir, time.strftime("match_%Y%m%d_%H%M%S.avi"))
            self._jpeg_writer = MjpegAviWriter(path, width, height, self.frame_fps)
            self._record_path = path
            self._record_frames = 0
            self._record_t0 = time.monotonic()
            self._record_error = None
        message = (
            f"Recording {path} at {width}x{height}, camera JPEG at up to {self.frame_fps} fps. "
            f"{self._disk_summary()}"
        )
        self.get_logger().info(message)
        return message

    def stop_recording(self) -> str:
        self._match_active = False
        with self._record_lock:
            writer = self._jpeg_writer
            path = self._record_path
            started = self._record_t0
            if writer is None:
                return "Not recording."
            self._jpeg_writer = None
            self._record_path = None
        elapsed = max(time.monotonic() - started, 0.001)
        writer.close(elapsed)
        frames = writer.frame_count
        stamped = restamp_avi(path, frames, elapsed)
        real_fps = frames / elapsed if elapsed > 0 else 0.0
        size_mb = os.path.getsize(path) / (1024 * 1024) if path and os.path.isfile(path) else 0
        message = (
            f"Saved {path} ({size_mb:.0f} MB, {frames} unique frames @ {real_fps:.1f} fps, "
            f"{elapsed:.1f} s real time, header {'updated' if stamped else 'NOT updated'}). "
            f"{self._disk_summary()}"
        )
        self.get_logger().info(message)
        return message

    def _infer_latest_frames(self):
        with requests.Session() as transport:
            while not self.stopped.is_set():
                try:
                    frame = self.frames.get(timeout=0.1)
                except queue.Empty:
                    continue
                try:
                    data = infer_frame(frame, self.api_key, self.api_url, transport)
                    put_latest(self.results, (data, None))
                except Exception as error:
                    safe = str(error).replace(self.api_key, "[REDACTED]")
                    put_latest(self.results, (None, safe))

    def _on_preview(self):
        with self._latest_lock:
            jpeg = self._latest_jpeg
        if not jpeg:
            return
        # Half-size preview only. The match file keeps the camera's full JPEG.
        flag = getattr(cv2, "IMREAD_REDUCED_COLOR_2", cv2.IMREAD_COLOR)
        encoded = np.frombuffer(jpeg, dtype=np.uint8)
        frame = cv2.imdecode(encoded, flag)
        if frame is None:
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._record_lock:
            recording = self._record_path is not None
        # Only feed Roboflow ~3 times per second (every 5th preview tick) so
        # inference never competes with 90 fps camera capture.
        tick = getattr(self, "_preview_tick", 0) + 1
        self._preview_tick = tick
        if self.run_model and tick % 5 == 0:
            put_latest(self.frames, frame)
        try:
            data, error = self.results.get_nowait()
            if error:
                self.status = f"Model inference failed: {error}"
                self.get_logger().error(self.status, throttle_duration_sec=2.0)
            else:
                self.latest_data = data
                self.status = None
                labels = [
                    f"{prediction.get('class', 'object')} {float(prediction.get('confidence', 0.0)):.0%}"
                    for prediction in (data or {}).get("predictions", [])
                    if isinstance(prediction, dict)
                ]
                self.label_pub.publish(String(data=", ".join(labels)))
        except queue.Empty:
            pass

        frame = annotate_frame(frame, self.latest_data)
        banner = f"{'REC ' if recording else ''}{self.capture_fps:.0f} fps"
        cv2.putText(
            frame,
            banner,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255) if recording else (255, 255, 255),
            2,
        )
        # The saved match video is full camera resolution. The ROS image is a
        # smaller copy so it does not flood the robot network.
        if tick % 4 == 0:
            preview = frame
            if frame.shape[1] > 960:
                scale = 960 / frame.shape[1]
                preview = cv2.resize(frame, (960, int(round(frame.shape[0] * scale))))
            self.image_pub.publish(_bgr_image_msg(preview, self.get_clock().now().to_msg()))
        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        )
        if ok:
            self.latest_jpeg.publish(encoded.tobytes())

        if not self.show_window:
            return
        try:
            cv2.imshow("VEX Model Output", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("Preview closed with q.")
                rclpy.shutdown()
        except cv2.error as error:
            self.get_logger().warning(f"Preview window unavailable: {error}")
            self.show_window = False

    def destroy_node(self):
        self.stop_recording()
        self.stopped.set()
        self._stop_pipeline(send_eos=False)
        self.latest_jpeg.close()
        if self.live_server is not None:
            self.live_server.shutdown()
        if self.camera is not None:
            self.camera.release()
        cv2.destroyAllWindows()
        if self.worker is not None and self.worker.is_alive():
            self.worker.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = CvCameraNode()
    except RuntimeError as error:
        message = str(error)
        api_key = os.environ.get("ROBOFLOW_API_KEY", "").strip()
        if api_key:
            message = message.replace(api_key, "[REDACTED]")
        print(message)
        rclpy.shutdown()
        raise SystemExit(1) from None
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Run the trained VEX detector on the camera plugged into this Jetson.

Frames are captured from a V4L2 camera (default /dev/video0) and sent to the
Roboflow serverless workflow. Nothing is written to disk. Watch the live
annotated image in a browser at http://<jetson-ip>:8080, or on the ROS topic
/cv/annotated_image. A preview window opens when show_window is true and
DISPLAY is set.

The HTTP workflow spec is inlined so this stays on the system Python that ROS
uses. Installing inference-sdk would pull NumPy 2 and break ROS packages.
"""

import sys

# Pip NumPy 2.x in ~/.local breaks ROS apt packages. Prefer system packages.
sys.path = [p for p in sys.path if ".local" not in p]

import base64
import os
import queue
import threading
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
from sensor_msgs.msg import Image
from std_msgs.msg import String

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


def annotate_frame(frame, data):
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
        x1 = int(prediction["x"] - prediction["width"] / 2)
        y1 = int(prediction["y"] - prediction["height"] / 2)
        x2 = int(prediction["x"] + prediction["width"] / 2)
        y2 = int(prediction["y"] + prediction["height"] / 2)
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
        body = (
            "<!doctype html><title>VEX camera</title>"
            "<body style=\"margin:0;background:#111;color:#eee;font-family:sans-serif\">"
            "<p style=\"padding:12px\">Live camera. Frames are not saved.</p>"
            "<img src=\"/stream\" style=\"width:100%;height:auto\">"
            "</body>"
        ).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
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
        self.declare_parameter("frame_width", 1280)
        self.declare_parameter("frame_height", 720)
        self.declare_parameter("show_window", False)
        self.declare_parameter("live_port", 8080)

        self.api_key = load_api_key()
        self.api_url = API_URL
        self.camera_device = int(self.get_parameter("camera_device").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.show_window = bool(self.get_parameter("show_window").value) and bool(
            os.environ.get("DISPLAY")
        )

        self.image_pub = self.create_publisher(Image, "/cv/annotated_image", 10)
        self.label_pub = self.create_publisher(String, "/cv/detection_labels", 10)
        self.frames = queue.Queue(maxsize=1)
        self.results = queue.Queue(maxsize=1)
        self.stopped = threading.Event()
        self.camera = None
        self.latest_data = None
        self.status = "Connecting to model..."
        self._publish_tick = 0
        self.latest_jpeg = LatestJpeg()
        self.live_port = int(self.get_parameter("live_port").value)
        self.live_server, live_error = start_live_server(self.live_port, self.latest_jpeg)

        self.worker = threading.Thread(target=self._infer_latest_frames, daemon=True)
        self.worker.start()
        self._open_camera()
        self.create_timer(1.0 / 15.0, self._on_timer)
        if self.live_server is None:
            self.get_logger().error(f"Live view port {self.live_port} failed: {live_error}")
        else:
            self.get_logger().info(
                f"Live view (nothing saved to disk): http://<jetson-ip>:{self.live_port}"
            )
        self.get_logger().info(
            f"VEX camera model {MODEL_ID} on device {self.camera_device}."
        )

    def _open_camera(self) -> bool:
        if self.camera is not None and self.camera.isOpened():
            return True
        camera = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        if not camera.isOpened():
            camera.release()
            self.get_logger().error(
                f"Could not open camera device {self.camera_device}. "
                "Plug the camera into the Jetson and confirm this user is in the video group "
                "(sudo usermod -aG video $USER, then log in again).",
                throttle_duration_sec=5.0,
            )
            return False
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.camera = camera
        width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Opened camera {self.camera_device} at {width}x{height}.")
        return True

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

    def _on_timer(self):
        if not self._open_camera():
            return
        success, frame = self.camera.read()
        if not success:
            self.get_logger().error(
                "Could not read a frame from the camera.",
                throttle_duration_sec=5.0,
            )
            self.camera.release()
            self.camera = None
            return

        put_latest(self.frames, frame.copy())
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
        if self.status:
            cv2.putText(
                frame,
                self.status[:80],
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

        # Full-rate 720p frames would saturate the robot's 100 Mbit link.
        self._publish_tick += 1
        if self._publish_tick % 3 == 0:
            preview = frame
            if frame.shape[1] > 640:
                scale = 640 / frame.shape[1]
                preview = cv2.resize(frame, (640, int(round(frame.shape[0] * scale))))
            self.image_pub.publish(_bgr_image_msg(preview, self.get_clock().now().to_msg()))
            ok, encoded = cv2.imencode(
                ".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), 70]
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
        self.stopped.set()
        self.latest_jpeg.close()
        if self.live_server is not None:
            self.live_server.shutdown()
        if self.camera is not None:
            self.camera.release()
        cv2.destroyAllWindows()
        if self.worker.is_alive():
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

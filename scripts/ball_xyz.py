from __future__ import annotations

import os
import sys
import types

# Pip NumPy 2.x in ~/.local breaks ROS apt packages (cv_bridge, cv2, matplotlib).
# Load system NumPy 1.x first; re-add ~/.local later only for ultralytics.
sys.path = [p for p in sys.path if ".local" not in p]


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
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Settings — change these for your setup
# ---------------------------------------------------------------------------
VEXU_HOME = os.environ.get("VEXU_HOME", os.path.expanduser("~/VEXU_GHOST"))
MODEL_PATH = os.path.join(VEXU_HOME, "best.pt")  # laptop: .pt  |  Orin: .engine

# YOLO class name to track (must match your trained model labels)
TARGET_CLASS = "red"  # change to "ball", "red", etc. if your model uses that

CONF_THRESHOLD = 0.5
YOLO_DEVICE = "cpu"  # laptop without NVIDIA GPU; use "0" if you have CUDA

# RealSense topic names (from: ros2 topic list)
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"

# Topic we publish to (other nodes / rviz can subscribe later)
OUTPUT_TOPIC = "/ball/position"
MAP_FRAME = "map"
OUTPUT_TOPIC_MAP = "/ball/position_map"
GROUND_Z_MAX = 0.15


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


class BallXYZNode(Node):
    def __init__(self):
        # Every ROS program is a "Node" with a name (shows up in ros2 node list)
        super().__init__("ball_xyz_node")

        self.get_logger().info("Starting ball_xyz_node")

        # Latest messages cached from callbacks (updated asynchronously)
        self.color_image = None
        self.depth_image = None
        self.color_frame_id = ""
        self.fx = self.fy = self.cx0 = self.cy0 = None

        #TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_link = self.create_publisher(PointStamped, "/ball/position_camera_link", 10)
        self.pub_map = self.create_publisher(PointStamped, OUTPUT_TOPIC_MAP, 10)
        # Load YOLO (same library as real.py, but .pt for laptop)
        if not os.path.isfile(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)

        _enable_ultralytics_imports()
        from ultralytics import YOLO

        self.get_logger().info(f"Loading model: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH, task="detect")
        warmup = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model.predict(warmup, device=YOLO_DEVICE, verbose=False)
        self.get_logger().info(f"YOLO ready (device={YOLO_DEVICE}, class={TARGET_CLASS})")

        # SUBSCRIBERS — "listen" to topics; callback runs when new message arrives
        
        self.create_subscription(Image, COLOR_TOPIC, self._on_color, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self._on_camera_info, 10)

        # PUBLISHER — "broadcast" ball position on a topic
        self.pub = self.create_publisher(PointStamped, OUTPUT_TOPIC, 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/ball/markers", 10)

        self.get_logger().info(f"Publishing XYZ on {OUTPUT_TOPIC}")

    def _on_camera_info(self, msg: CameraInfo):
        """Camera intrinsics — needed for pixel -> XYZ. Received once or rarely."""
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx0 = msg.k[2]
        self.cy0 = msg.k[5]

    def _on_depth(self, msg: Image):
        """Depth aligned to color — same pixel (u,v) = same physical point."""
        self.depth_image = image_msg_to_depth_u16(msg)

    def _on_color(self, msg: Image):
        """
        Main loop trigger: each new color frame, try to find ball and publish XYZ.
        (Simple pattern: process when color updates; depth/info must already be cached.)
        """
        self.color_image = image_msg_to_bgr8(msg)
        self.color_frame_id = msg.header.frame_id
        self._process_frame()

    def _process_frame(self):
        if self.color_image is None or self.depth_image is None:
            return
        if None in (self.fx, self.fy, self.cx0, self.cy0):
            return

        results = self.model.predict(
            self.color_image,
            device=YOLO_DEVICE,
            conf=CONF_THRESHOLD,
            verbose=False,
        )

        detections = []  # (depth_m, cx, cy, class_name, confidence)

        for result in results:
            if result.boxes is None or len(result.boxes) == 0:
                continue
            for box in result.boxes:
                class_name = result.names[int(box.cls)]
                # if class_name != TARGET_CLASS:
                #     continue

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                depth_m = median_depth_m(self.depth_image, cx, cy)
                if depth_m is None:
                    continue

                detections.append((depth_m, cx, cy, class_name, float(box.conf)))

        if not detections:
            clear = Marker()
            clear.header.stamp = self.get_clock().now().to_msg()
            clear.header.frame_id = "camera_link"
            clear.action = Marker.DELETEALL
            self.marker_pub.publish(MarkerArray(markers=[clear]))
            return

        closest_idx = min(range(len(detections)), key=lambda i: detections[i][0])

        stamp = self.get_clock().now().to_msg()
        try:
            transform = self.tf_buffer.lookup_transform("camera_link", self.color_frame_id, rclpy.time.Time(), Duration(seconds=1.0))
            map_transform = self.tf_buffer.lookup_transform(MAP_FRAME, self.color_frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            marker_array = MarkerArray()
            closest_optical = None
            closest_link = None
            closest_map = None
            log_parts = []
            for idx, (depth_m, cx, cy, class_name, conf) in enumerate(detections):
                msg = PointStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = self.color_frame_id  # usually camera_color_optical_frame
                x, y, z = pixel_to_xyz(cx, cy, depth_m, self.fx, self.fy, self.cx0, self.cy0)
                msg.point.x = x
                msg.point.y = y
                msg.point.z = z
                points_in_link = do_transform_point(msg, transform)
                marker = Marker()
                marker.header.frame_id = "camera_link"
                marker.type = Marker.SPHERE
                marker.action=Marker.ADD
                marker.pose.position=points_in_link.point
                if class_name == "red":
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
                elif class_name == "blue":
                    marker.color.r, marker.color.g, marker.color.b = 0.1, 0.4, 1.0
                else:
                    marker.color.r, marker.color.g, marker.color.b = 0.8, 0.8, 0.8
                marker.color.a = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.id = idx
                marker.header.stamp = stamp
                marker.pose.orientation.w = 1.0
                marker_array.markers.append(marker)
                if idx == closest_idx:
                    closest_optical = msg
                    closest_link = points_in_link
                    closest_map = do_transform_point(msg, map_transform)
                    closest_map.header.stamp = stamp
                log_parts.append(f"{class_name}({conf:.2f})@({x:.2f},{y:.2f},{z:.2f})")
            if closest_optical is not None and closest_link is not None:
                self.pub.publish(closest_optical)
                self.pub_link.publish(closest_link)
                self.pub_map.publish(closest_map)
            self.marker_pub.publish(marker_array)
            self.get_logger().info(
                f"{len(detections)} object(s): " + ", ".join(log_parts),
                throttle_duration_sec=0.5,
            )
            self.get_logger().info(f"Published marker in camera_link frame")
        except TransformException as e:
            self.get_logger().warning(f"Transform exception: {e}", throttle_duration_sec=2.0)

        self.get_logger().info(
            f"{class_name} ({conf:.2f}) pixel=({cx},{cy}) "
            f"XYZ=({x:.3f}, {y:.3f}, {z:.3f}) m  frame={self.color_frame_id}",
            throttle_duration_sec=0.5,
        )


def main():
    rclpy.init()
    node = BallXYZNode()
    try:
        # spin() = "keep running and call my subscribers when messages arrive"
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

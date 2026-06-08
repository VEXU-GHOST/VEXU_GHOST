#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_perception.cv_common import (
    YoloDetector, default_model_path,
    image_msg_to_bgr8, image_msg_to_depth_u16, pixel_to_xyz,
)
from geometry_msgs.msg import PointStamped
from push_back_cv.msg import CvDetection
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

class CvDetector(Node):
    COLOR_TOPIC = "/camera/camera/color/image_raw"
    DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"  # must be aligned!
    INFO_TOPIC = "/camera/camera/color/camera_info"

    def __init__(self):
        # Every ROS program is a "Node" with a name (shows up in ros2 node list)
        super().__init__("cv_detector")

        self.get_logger().info("Starting cv_detector")

        # Latest messages cached from callbacks (updated asynchronously)
        self.color_image = None
        self.depth_image = None
        self.color_frame_id = ""
        self.fx = self.fy = self.cx0 = self.cy0 = None

        # SUBSCRIBERS — "listen" to topics; callback runs when new message arrives
        
        self.create_subscription(Image, self.COLOR_TOPIC, self._on_color, 10)
        self.create_subscription(Image, self.DEPTH_TOPIC, self._on_depth, 10)
        self.create_subscription(CameraInfo, self.INFO_TOPIC, self._on_camera_info, 10)
        self.detector = YoloDetector(default_model_path(), "cpu", 0.5)
        self.map_frame = "map"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(CvDetection, "/cv/detection", 10)


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
        detections = self.detector.detect(self.color_image, self.depth_image)
        if not detections:
            return
        closest = min(detections, key=lambda d: d.depth_m)
        stamp = self.get_clock().now().to_msg()

        try:
            map_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.color_frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warning(f"TF failed: {exc}", throttle_duration_sec=2.0)
            return

        optical = PointStamped()
        optical.header.stamp = stamp
        optical.header.frame_id = self.color_frame_id
        optical.point.x, optical.point.y, optical.point.z = pixel_to_xyz(
            closest.cx, closest.cy, closest.depth_m,
            self.fx, self.fy, self.cx0, self.cy0,
        )

        map_point = do_transform_point(optical, map_transform)

        out = CvDetection()
        out.x = map_point.point.x
        out.y = map_point.point.y
        out.z = map_point.point.z
        out.is_red = closest.is_red
        out.confidence = closest.confidence
        self.pub.publish(out)

        self.get_logger().info(
            f"published {closest.class_name} map=({out.x:.2f}, {out.y:.2f}, {out.z:.2f})",
            throttle_duration_sec=0.5,
        )

def main():
    rclpy.init()
    node = CvDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
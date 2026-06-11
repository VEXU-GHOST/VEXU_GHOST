#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_perception.cv_common import (
    YoloDetector, default_model_path,
    image_msg_to_bgr8, image_msg_to_depth_u16, pixel_to_xyz,
)
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from push_back_cv.msg import CvDetection, CvDetectionArray
from visualization_msgs.msg import Marker, MarkerArray


class CvDetectorArray(Node):
    COLOR_TOPIC = "/camera/camera/color/image_raw"
    DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"  # must be aligned!
    INFO_TOPIC = "/camera/camera/color/camera_info"

    def __init__(self):
        # Every ROS program is a "Node" with a name (shows up in ros2 node list)
        super().__init__("cv_detector_array")

        self.get_logger().info("Starting cv_detector_array")

        # Latest messages cached from callbacks (updated asynchronously)
        self.color_image = None
        self.depth_image = None
        self.color_frame_id = ""
        self.fx = self.fy = self.cx0 = self.cy0 = None

        # SUBSCRIBERS — "listen" to topics; callback runs when new message arrives
        
        self.create_subscription(Image, self.COLOR_TOPIC, self._on_color, 10)
        self.create_subscription(Image, self.DEPTH_TOPIC, self._on_depth, 10)
        self.create_subscription(CameraInfo, self.INFO_TOPIC, self._on_camera_info, 10)
        self.pub = self.create_publisher(CvDetectionArray, "/cv/detections", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/cv/markers", 10)
        self.marker_frame = "camera_link"
        self.detector = YoloDetector(default_model_path(), "cpu", 0.5)
        self.map_frame = "map"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


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

    def _make_marker(self, idx: int, stamp, position, class_name: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = stamp
        marker.ns = "blocks"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        if class_name == "red":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        elif class_name == "blue":
            marker.color.r, marker.color.g, marker.color.b = 0.1, 0.4, 1.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.8, 0.8, 0.8
        marker.color.a = 1.0
        return marker
    
    def _process_frame(self):
        if self.color_image is None or self.depth_image is None:
            return
        if None in (self.fx, self.fy, self.cx0, self.cy0):
            return
        detections = self.detector.detect(self.color_image, self.depth_image)
        if not detections:
            empty = CvDetectionArray()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = self.map_frame
            self.pub.publish(empty)

            if self.marker_pub.get_subscription_count() > 0:
                clear = Marker()
                clear.header.stamp = empty.header.stamp
                clear.header.frame_id = self.marker_frame
                clear.action = Marker.DELETEALL
                self.marker_pub.publish(MarkerArray(markers=[clear]))
            return
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
        
        want_markers = self.marker_pub.get_subscription_count() > 0
        link_transform = None
        marker_array = MarkerArray()

        if want_markers:
            try:
                link_transform = self.tf_buffer.lookup_transform(
                    self.marker_frame,
                    self.color_frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
            except TransformException as exc:
                self.get_logger().warning(f"Marker TF failed: {exc}", throttle_duration_sec=2.0)
                want_markers = False

        array_msg = CvDetectionArray()
        array_msg.header.stamp = stamp
        array_msg.header.frame_id = self.map_frame

        for det in detections:
            optical = PointStamped()
            optical.header.stamp = stamp
            optical.header.frame_id = self.color_frame_id
            optical.point.x, optical.point.y, optical.point.z = pixel_to_xyz(
                det.cx, det.cy, det.depth_m,
                self.fx, self.fy, self.cx0, self.cy0,
            )
            map_point = do_transform_point(optical, map_transform)

            out = CvDetection()
            out.x = map_point.point.x
            out.y = map_point.point.y
            out.z = map_point.point.z
            out.is_red = det.is_red
            out.confidence = det.confidence
            array_msg.detections.append(out)
            if want_markers and link_transform is not None:
                link_point = do_transform_point(optical, link_transform)
                marker_array.markers.append(
                    self._make_marker(len(marker_array.markers), stamp, link_point.point, det.class_name)
                )

        self.pub.publish(array_msg)
        if want_markers and marker_array.markers:
            self.marker_pub.publish(marker_array)
        self.get_logger().info(
            f"published {len(array_msg.detections)} detection(s)",
            throttle_duration_sec=0.5,
        )

def main():
    rclpy.init()
    node = CvDetectorArray()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import time


class RealSenseYOLOCombined(Node):
    def __init__(self):
        super().__init__('realsense_yolo_combined_node')
        self.model = YOLO("/home/ghost/VEXU_GHOST/best.pt")
        self.bridge = CvBridge()

        self.color_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/detected_objects_marker', 10)
        self.processed_image_pub = self.create_publisher(Image, '/yolo_processed_image', 10)
        self.xy_publisher = self.create_publisher(Float64MultiArray, '/object_xy_positions', 10)

        self.get_logger().info("RealSense YOLO Combined node initialized.")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f"Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.try_process()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        self.try_process()

    def try_process(self):
        if self.color_image is None or self.depth_image is None or None in (self.fx, self.fy, self.cx, self.cy):
            return

        frame = self.color_image.copy()
        results = self.model(frame)

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for box in boxes:
                class_id = int(box.cls)
                class_name = result.names[class_id]

                if class_name != "mobile-goal":
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx_px = int((x1 + x2) / 2)
                cy_px = int((y1 + y2) / 2)

                h, w = self.depth_image.shape
                x_start = max(cx_px - 2, 0)
                x_end = min(cx_px + 3, w)
                y_start = max(cy_px - 2, 0)
                y_end = min(cy_px + 3, h)

                roi = self.depth_image[y_start:y_end, x_start:x_end]
                valid_depths = roi[roi > 0].flatten()
                if valid_depths.size < 3:
                    self.get_logger().warn(f"No valid depth values for {class_name}, skipping.")
                    continue

                sorted_depths = np.sort(valid_depths)
                num_low = max(1, int(0.2 * len(sorted_depths)))
                lowest_20 = sorted_depths[:num_low]
                depth_m = np.mean(lowest_20) / 1000.0  # Convert mm to meters

                rs_X = (cx_px - self.cx) * depth_m / self.fx
                rs_Y = (cy_px - self.cy) * depth_m / self.fy
                rs_Z = depth_m

                # RealSense -> RViz conversion
                rviz_x = rs_Z
                rviz_y = -rs_X

                label = f"{class_name}: {box.conf.item():.2f}, Z: {rs_Z:.2f}m"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                self.get_logger().info(
                    f"Detected {class_name} | Conf: {box.conf.item():.2f} | RViz X: {rviz_x:.2f}m, Y: {rviz_y:.2f}m"
                )

                # Publish marker and coordinate array
                self.publish_marker(rviz_x, rviz_y, 0.0)

                xy_array_msg = Float64MultiArray()
                xy_array_msg.data = [rviz_x, rviz_y]
                self.xy_publisher.publish(xy_array_msg)
                self.get_logger().info(f"Published X: {rviz_x:.2f}, Y: {rviz_y:.2f} on /object_xy_positions")

        # Save image
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(f"/home/ghost/VEXU_GHOST/runs/detect/predict_{timestamp}.jpg", frame)

        # Publish image
        processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.header.frame_id = "camera_color_optical_frame"
        self.processed_image_pub.publish(processed_msg)

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detections"
        marker.id = int(time.time() * 1000) % 100000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYOLOCombined()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

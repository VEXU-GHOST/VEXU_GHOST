import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import time


class RealSenseYOLO(Node):
    def __init__(self):
        super().__init__('realsense_yolo_node')
        self.model = YOLO("/home/ghost/VEXU_GHOST/best.pt")
        self.bridge = CvBridge()

        self.color_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/detected_objects_marker', 10)

        self.get_logger().info("RealSense YOLO node initialized.")

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f"Camera intrinsics received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def color_callback(self, msg):
        print("recieved color image!")
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.try_process()

    def depth_callback(self, msg):
        print("recieved depth image!")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        self.try_process()

    def try_process(self):
        if self.color_image is None:
            self.get_logger().warn("Skipping processing — color image not yet received.")
            return
        if self.depth_image is None:
            self.get_logger().warn("Skipping processing — depth image not yet received.")
            return
        if None in (self.fx, self.fy, self.cx, self.cy):
            self.get_logger().warn("Skipping processing — camera intrinsics not yet received.")
            return

        self.get_logger().info("Processing frame...")

        frame = self.color_image.copy()
        results = self.model(frame)

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                class_name = result.names[int(box.cls)]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                h, w = self.depth_image.shape
                box_height = y2 - y1

                if class_name == "rings-goals":
                    lower_y1 = y2 - int(0.2 * box_height)
                    upper_y1 = y2
                    box_color = (0, 255, 255)
                else:
                    lower_y1 = y2 - int(0.30 * box_height)
                    upper_y1 = y2 - int(0.05 * box_height)
                    box_color = (0, 255, 0)

                cv2.rectangle(frame, (x1, lower_y1), (x2, upper_y1), box_color, 2)

                cx_px = int((x1 + x2) / 2)
                cy_px = int((lower_y1 + upper_y1) / 2)

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
                depth_m = np.mean(lowest_20) / 1000.0

                X = (cx_px - self.cx) * depth_m / self.fx
                Y = (cy_px - self.cy) * depth_m / self.fy
                Z = depth_m

                label = f"{class_name}: {box.conf.item():.2f}, Z: {Z:.2f}m"
                cv2.putText(frame, label, (x1, lower_y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                self.get_logger().info(
                    f"Detected {class_name} | Confidence: {box.conf.item():.2f} | X: {X:.2f}m, Y: {Y:.2f}m, Z: {Z:.2f}m"
                )

                self.publish_marker(X, Y, Z)

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        #cv2.imwrite(f"/home/ghost/VEXU_GHOST/runs/detect/predict_{timestamp}.jpg", frame)

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
    node = RealSenseYOLO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

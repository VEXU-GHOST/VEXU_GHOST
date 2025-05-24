import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
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

        self.get_logger().info("RealSense YOLO node initialized.")

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
        if self.color_image is None or self.depth_image is None:
            return
        if None in (self.fx, self.fy, self.cx, self.cy):
            return

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
                    # Bottom 20%
                    lower_y1 = y2 - int(0.2 * box_height)
                    upper_y1 = y2
                    box_color = (0, 255, 255)  # yellow-ish for rings-goals
                else:
                    # Bottom 5% to 30%
                    lower_y1 = y2 - int(0.30 * box_height)  # 30% above bottom
                    upper_y1 = y2 - int(0.05 * box_height)  # 5% above bottom
                    box_color = (0, 255, 0)  # green for others

                # Draw rectangle on the selected vertical range
                cv2.rectangle(frame, (x1, lower_y1), (x2, upper_y1), box_color, 2)

                # Center pixel of the selected region
                cx_px = int((x1 + x2) / 2)
                cy_px = int((lower_y1 + upper_y1) / 2)

                # Define 5x5 region around the center pixel
                x_start = max(cx_px - 2, 0)
                x_end = min(cx_px + 3, w)
                y_start = max(cy_px - 2, 0)
                y_end = min(cy_px + 3, h)

                roi = self.depth_image[y_start:y_end, x_start:x_end]
                valid_depths = roi[roi > 0].flatten()
                if valid_depths.size < 3:
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
                    f"Detected {class_name} | Confidence: {box.conf.item():.2f} | X: {X:.2f}m, Z: {Z:.2f}m")

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        cv2.imwrite(f"/home/ghost/VEXU_GHOST/runs/detect/predict_{timestamp}.jpg", frame)


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

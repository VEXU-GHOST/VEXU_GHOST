import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os


class RealSenseImageSaver(Node):
    def __init__(self):
        super().__init__('realsense_image_saver_node')
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.get_logger().info("RealSense Image Saver node initialized.")

        # Optional: save path
        self.save_dir = "/home/ghost/VEXU_GHOST/saved_images"
        os.makedirs(self.save_dir, exist_ok=True)

    def color_callback(self, msg):
        self.get_logger().info("Received color image.")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(self.save_dir, f"color_{timestamp}.jpg")
        cv2.imwrite(filename, image)
        self.get_logger().info(f"Saved image to {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

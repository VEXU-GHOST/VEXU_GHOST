import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import time  # Add this line

class RealSenseYOLO(Node):
    def __init__(self):
        super().__init__('realsense_yolo_node')

        # Initialize YOLO model
        self.model = YOLO("/home/ghost/VEXU_GHOST/best.pt")

        # Initialize OpenCV Bridge
        self.bridge = CvBridge()

        # Create a subscription to the ROS2 topic /camera/camera/color/image_raw
        self.subscription = self.create_subscription(
            Image,  # Message type
            '/camera/camera/color/image_raw',  # Topic name
            self.image_callback,  # Callback function
            10  # Queue size
        )

    def image_callback(self, msg):
        # Convert the ROS2 Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model(frame)

        # Draw results
        for result in results:
            frame = result.plot()

        # Generate timestamp for the image file name
        timestamp = time.strftime("%Y%m%d-%H%M%S")

        # Save the image with timestamp in the name
        cv2.imwrite(f"/home/ghost/VEXU_GHOST/runs/detect/predict_{timestamp}.jpg", frame)

def main(args=None):
    rclpy.init(args=args)

    # Initialize the RealSense YOLO Node
    node = RealSenseYOLO()

    try:
        # Spin the ROS2 node (this will keep it running and handling messages)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown the node
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
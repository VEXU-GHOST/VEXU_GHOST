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
import os

class RealSenseYOLOCombined(Node):
    def __init__(self):
        super().__init__('realsense_yolo_combined_node')
        
        # 1. Load the Engine
        engine_path = "/home/ghost/VEXU_GHOST/best.engine"
        self.model = YOLO(engine_path, task='detect')
        
        # 2. PERFORM WARMUP
        self.get_logger().info("Warming up TensorRT engine...")
        warmup_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # We use predict here just to prime the hardware
        self.model.predict(warmup_frame, device=0, verbose=False)
        self.get_logger().info("Warmup complete. Jerry is active.")

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None
        
        # Counter for unique image filenames
        self.img_counter = 0

        # Subscriptions (Note: Ensure camera aligns depth to color)
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/detected_objects_marker', 10)
        self.processed_image_pub = self.create_publisher(Image, '/yolo_processed_image', 10)
        self.xy_publisher = self.create_publisher(Float64MultiArray, '/object_xy_positions', 10)

       

    def camera_info_callback(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.try_process()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def try_process(self):
        start_time = time.time()
        if self.color_image is None or self.depth_image is None or None in (self.fx, self.fy, self.cx, self.cy):
            return

        # TIME: Frame copy
        t0 = time.time()
        frame = self.color_image.copy()
        copy_time = (time.time() - t0) * 1000
        
        # TIME: YOLO tracking
        t1 = time.time()
        results = self.model.predict(frame, device=0, conf=0.5, verbose=True)
        yolo_time = (time.time() - t1) * 1000

        # TIME: Depth processing and publishing (for all objects)
        t2 = time.time()
        found_any = False
        num_objects = 0
    
        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue
        
            found_any = True
            for i,box in enumerate(boxes):
                num_objects += 1
                obj_id = i
                class_id = int(box.cls)
                class_name = result.names[class_id]
                
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx_px, cy_px = (x1 + x2) // 2, (y1 + y2) // 2

                # Depth logic
                h, w = self.depth_image.shape

                # Creating 5x5 region of interest
                x_start, x_end = max(cx_px - 2, 0), min(cx_px + 3, w)
                y_start, y_end = max(cy_px - 2, 0), min(cy_px + 3, h)
                # Creates 2D list
                roi = self.depth_image[y_start:y_end, x_start:x_end]

                # Filter out 0 depth and flatten to 1D
                valid_depths = roi[roi > 0].flatten()

                if valid_depths.size >= 3:
                    depth_m = np.median(valid_depths) / 1000.0
                    rs_X = (cx_px - self.cx) * depth_m / self.fx
                    rs_Z = depth_m
                    rviz_x, rviz_y = rs_Z, -rs_X

                    self.publish_marker(rviz_x, rviz_y, 0.0, obj_id)
                    xy_msg = Float64MultiArray()
                    xy_msg.data = [rviz_x, rviz_y, float(obj_id)]
                    self.xy_publisher.publish(xy_msg)

        depth_publish_time = (time.time() - t2) * 1000
        
        total_time = (time.time() - start_time) * 1000
    
        # Detailed logging
        self.get_logger().info(
            f"[{num_objects} objs] "
            f"Copy: {copy_time:.1f}ms | "
            f"YOLO: {yolo_time:.1f}ms | "
            f"Depth+Pub: {depth_publish_time:.1f}ms | "
            f"TOTAL: {total_time:.1f}ms"
        )

    def publish_marker(self, x, y, z, obj_id):
        marker = Marker()
        marker.header.frame_id = "camera_link" # Changed to camera_link for better alignment
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = obj_id
        marker.type = Marker.SPHERE
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r, marker.color.a = 1.0, 1.0
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
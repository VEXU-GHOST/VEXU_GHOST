from ultralytics import YOLO
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D


class YoloSegmentNode(Node):

    def __init__(self):
        super().__init__("yolo_segment_node")
        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, "camera/image_raw", self.image_callback, 10
        )
        self.publisher = self.create_publisher(Image, "yolo_detection_plot", 10)
        self.classes_publisher = self.create_publisher(
            Detection2DArray, "yolo_boxes", 10
        )
        self.get_logger().info("YoloSegmentNode initialized.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(cv_image)
        result = results[0]
        # # Create an empty image to combine the segmented masks
        if result is None:
            return None
        # # all classes that can be outputted by the yolo model
        boxes = result.boxes.cpu().xywh
        if boxes is None:
            return
        list_yolo_segments = []
        for box in boxes:
            print("Box(Test): ", box)   
            detection2d = Detection2D()
            detection2d.bbox = BoundingBox2D()  
            detection2d.bbox.center.x = box[0]
            detection2d.bbox.center.y = box[1]
            detection2d.bbox.size_x = box[2]
            detection2d.bbox.size_y = box[3]
            list_yolo_segments.append(box.flatten().tolist())
        # print("Segments(Test): ", list_yolo_segments)
        result_boxes_classes = result.boxes.cpu().numpy().cls
        detection2d_array = Detection2DArray()
        detection2d_array.detections = []
        classes_inferred = [result.names[i] for i in result_boxes_classes]
        # print("Classes(Test): ", classes_inferred)
        vis_frame = result.plot()

        # # Convert the combined mask to image message
        image_message = self.bridge.cv2_to_imgmsg(vis_frame, encoding="bgr8")
        self.publisher.publish(image_message)


def main():
    rclpy.init()
    yolo_segment_node = YoloSegmentNode()
    rclpy.spin(yolo_segment_node)
    yolo_segment_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

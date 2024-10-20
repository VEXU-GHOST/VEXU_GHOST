from ultralytics import YOLO
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, Pose2D, Point2D
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber

class YoloSegmentNode(Node):

    def __init__(self):
        super().__init__("yolo_segment_node")
        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()
        self.subscription = Subscriber(
            self, Image, "camera/camera/color/image_raw"
        )
        self.pcd_subscription = Subscriber(self, PointCloud2, "camera/camera/depth/color/points")
        self.publisher = self.create_publisher(Image, "yolo_detection_plot", 10)
        self.ts = ApproximateTimeSynchronizer(
           [self.subscription, self.pcd_subscription], 10, 0.1, allow_headerless=True
        )
        self.ts.registerCallback(self.image_callback)
        self.classes_publisher = self.create_publisher(
            Detection2DArray, "yolo_boxes", 10
        )
        self.get_logger().info("YoloSegmentNode initialized.")

    def image_callback(self, msg: Image, pcd: PointCloud2):
        self.get_logger().info("Synchronized Callback")

        # Node use the positions returned by the detections and get relevant points

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(cv_image)
        result = results[0]
        # # Create an empty image to combine the segmented masks
        if result is None:
            return None
        # # all classes that can be outputted by the yolo model
        boxes = result.boxes.cpu().xywh
        print("Boxes: ", boxes)
        if boxes is None:
            return
        list_yolo_segments = []
        for box in boxes:
            print("Box(Test): ", box)   
            detection2d = Detection2D()
            detection2d.bbox = BoundingBox2D()  
            # detection2d.bbox.center = Pose2D()
            point = Point2D()
            point.x = float(box[0])
            point.y = float(box[1])
            pose = Pose2D()
            pose.position = point
            print(type(point))
            detection2d.bbox.center = pose
            detection2d.bbox.size_x = float(box[2])
            detection2d.bbox.size_y = float(box[3])
            list_yolo_segments.append(detection2d)
        print("Segments(Test): ", list_yolo_segments)
        result_boxes_classes = result.boxes.cpu().numpy().cls
        detection2d_array = Detection2DArray()
        detection2d_array.detections = []
        classes_inferred = [result.names[i] for i in result_boxes_classes]
        # print("Classes(Test): ", classes_inferred)
        vis_frame = result.plot()

        # # Convert the combined mask to image message
        image_message = self.bridge.cv2_to_imgmsg(vis_frame, encoding="bgr8")
        self.publisher.publish(image_message)
        self.classes_publisher.publish(detection2d_array)


def main():
    rclpy.init()
    yolo_segment_node = YoloSegmentNode()
    rclpy.spin(yolo_segment_node)
    yolo_segment_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

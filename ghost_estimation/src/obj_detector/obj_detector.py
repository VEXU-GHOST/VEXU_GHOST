#!/usr/bin/env python3

#import pyrealsense2 as rs
import numpy as np
import cv2
import imutils
import time
import rclpy # Python library for ROS 2
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import PoseWithCovariance
from ghost_msgs.msg import CVObjList
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster

cov_ratio = 1

class Object:
    def __init__(this, x, y, radius):
        this.x = x
        this.y = y
        this.radius = radius
        this.distance = 0.0
        this.distance_covariance = 0.0
        this.angle = 0.0
        this.angle_covariance = 0.0

    

class ObjectDetector:
    def __init__(this, hsize, vsize, hfov, vfov, lower_color, upper_color):
        this.hsize = hsize
        this.vsize = vsize
        this.hfov = hfov
        this.vfov = vfov
        this.lower_color = lower_color
        this.upper_color = upper_color

    def filterObject(this, image):
        '''
        Filters the image using cv2 for object colors (yellow)

        Parameters: 
            image - (numpy array) image from camera
            
        Returns: 
            angle covariance - (float) covariance of the angle
        '''
        image = cv2.GaussianBlur(image, (11, 11), 0)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image = cv2.inRange(image, this.lower_color, this.upper_color)
        image = cv2.dilate(image, None, iterations=2)       
        image = cv2.erode(image, None, iterations=2)
        return image

    def findObjects(this, image):
        '''
        Finds objects in the image using cv2

        Parameters: 
            image - (numpy array) filtered image from camera
            
        Returns: 
            objects - (list(Objects)) list of objects found
        '''
        contours = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        radius = 0

        objects = []
        for contour in contours:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > 10:
                objects.append(Object(x, y, radius))
        return objects

    def addObjectToDisplay(this, image, object: Object, text: bool):
        '''
        Adds object info to the camera image

        Parameters: 
            object - (Object) object to add info for
            image - (numpy array) image to add info to
            text - (boolean) whether to add text or only boxes
            
        Returns: 
            image - (numpy array) covariance of the angle
        '''
        meter_to_inch = 39.3701
        if object.radius > 10:
            startpt = (int(object.x) - int(object.radius), int(object.y) - int(object.radius))
            endpt = (int(object.x) + int(object.radius), int(object.y) + int(object.radius))
            image = cv2.rectangle(image, startpt, endpt, (255, 255, 0), 1)
            if text:
                distance = round(object.distance,3)
                angle = round(object.angle,3)
                text_dist = f'distance: {distance}\"'
                text_angle = f'angle: {angle}'
                textList = [text_angle,text_dist]
                for i in range(len(textList)):
                    text_position = (int(object.x) + int(object.radius), int(object.y) + int(object.radius) + i*15)
                    image = cv2.putText(image, textList[i], text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1, cv2.LINE_AA)
        return image

    def calcDistCovariance(self, object, depth_image, unit_conversion) -> float:
        '''
        Calculates distance covariance by finding the difference 
        in distance at the top/bottom edges of the object 

        Parameters: 
            object - (Object) object to calculate angle covariance for
            depth_image - (np.array) depth image from camera
            unit_conversion - (float) ratio to convert mm to m 
            
        Returns: 
            angle covariance - (float) covariance of the angle
        '''
        depth_max_y = depth_image[int(min(object.y + object.radius/2, 479))][int(object.x)] * unit_conversion
        depth_min_y = depth_image[int(max(object.y - object.radius/2, 0))][int(object.x)] * unit_conversion
        return float(abs((depth_max_y - depth_min_y) * cov_ratio))

    def calcAngle(this, object, hsize=640, hfov=69) -> float:
        '''
        Calculates the angle of a object from the forward (x) axis

        Parameters: 
            object - (Object) object to calculate angle for
            hsize - (int) horizontal size of image in pixels
            hfov - (int) horizontal size of camera view in degrees
            
        Returns: 
            angle - (float) angle from the forward axis
        '''
        angle = -1*((object.x - hsize/2)/(hsize/2))*(hfov/2)*math.pi/180
        return angle

    def calcAngleCovariance(this, object, hsize=640, hfov=69) -> float:
        '''
        Calculates angle covariance by finding the difference 
        in angle at the left/right edges of the object 

        Parameters: 
            object - (Object) object to calculate angle covariance for
            hsize - (int) horizontal size of image in pixels
            hfov - (int) horizontal size of camera view in degrees
            
        Returns: 
            angle covariance - (float) covariance of the angle
        '''
        angle_min = ((object.x + object.radius/2 - hsize/2)/(hsize/2))*(hfov/2)*math.pi/180
        angle_max = ((object.x - object.radius/2 - hsize/2)/(hsize/2))*(hfov/2)*math.pi/180
        return abs((angle_max - angle_min) * cov_ratio)

    def detectObjects(this, color_image: np.array, depth_image: np.array):
        '''
        Detects objects in the given image

        Parameters: 
            color_image - (numpy array) image from camera
            depth_image - (numpy array) depth image from camera

        Returns: 
            color_image - (numpy array) image with added text/boxes around objects
            objects -       (list(objects) list of objects detected
        '''
        try:
            #starttime = time.perf_counter()
            unit_conversion = 1/1000
            object_image = this.filterObject(color_image)
            objects = this.findObjects(object_image)
            for d in objects:
                if not d:
                    continue
                try:
                    if not depth_image:
                        d.distance = 1000
                except:
                    d.distance = float(depth_image[int(d.y)][int(d.x)] * unit_conversion)
                    d.distance_covariance = this.calcDistCovariance(d, depth_image, unit_conversion)
                    d.angle = this.calcAngle(d)
                    d.angle_covariance = this.calcAngleCovariance(d)
                color_image = this.addObjectToDisplay(color_image, d, True)
            #fps = 1/(time.perf_counter() - starttime)
            #color_image = cv2.putText(color_image, f"fps: {fps:.2f}", (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1, cv2.LINE_AA)
            return color_image, objects
        finally:
            a=0
        #return object_image
        #object_detector.stopDisplay

def initCamera():
    '''
    Initializes object detector object

    Returns: 
        object_detector - ObjectDetector object
    '''
    hsize = 640
    vsize = 480
    hfov = 69
    vfov = 42
    lower_color = (25,70,100)
    upper_color = (80,255,255)
    object_detector = ObjectDetector(hsize, vsize, hfov, vfov, lower_color, upper_color)
    return object_detector

class ObjectDetectorNode(Node):
    def __init__(self):
        '''
        Constructor
        Creates image and depth subscribers
        Creates image and object publishers
        Initializes all variables saved in the node object and timer
        '''
        super().__init__('object_detector_node')
        self.image_subscription = self.create_subscription(
            Image,                                         
            '/camera/color/image_raw',                     
            self.image_listener_callback,                  
            10)                                            
        self.depth_subscription = self.create_subscription(
            Image,                                         
            '/camera/depth/image_rect_raw',                
            self.depth_listener_callback,                  
            10)                                            
        
        self.image_publisher_ = self.create_publisher(
            Image, 
            'cv_frames', 
            10)
        self.object_detector = initCamera()
        self.br = CvBridge()
        self.cv_image = None
        self.depth_image = None
        
        self.object_publisher_ = self.create_publisher(
            CVObjList, 
            'cv_objects', 
            10)
        self.objects = []
        
        self.log_string = ''
        self.header = None
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback) #timer to call publishing function

    def timer_callback(self):
        '''
        Callback function when the timer is triggered
        Publishes the image and objects saved in the node object
        '''
        self.log_string = ''
        
        #publish image with boxes around objects
        try:
            if not self.cv_image:
                self.log_string += 'Video frame not found, '
        except:
            self.image_publisher_.publish(self.br.cv2_to_imgmsg(self.cv_image,encoding="rgb8"))
            self.log_string += 'Publishing video frame, '
        
        #publish ObjectList
        if not self.objects:
            self.log_string += 'Objects not found, '
        else: 
            #ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:="640X480X15" enable_depth:=true depth_module.profile:="640X480X15"
            cv_object_list = CVObjList()
            cv_object_list.header = self.header
            for d in self.objects:
                cv_object = PoseWithCovariance()
                cv_object.pose.position.y = d.distance * math.sin(d.angle)
                cv_object.pose.position.x = d.distance * math.cos(d.angle)
                cv_object.pose.position.z = -0.38 #-15 in m # 0.0
                r = R.from_euler('xyz', [0, 0, d.angle])
                cv_object.pose.orientation.x = r.as_quat()[0]#0.0#0.707
                cv_object.pose.orientation.y = r.as_quat()[1]#0.0
                cv_object.pose.orientation.z = r.as_quat()[2]#0.0
                cv_object.pose.orientation.w = r.as_quat()[3]#1.0#0.707
                covariance = np.zeros(36)
                covariance[0] = d.distance_covariance**2
                covariance[1+6*1] = (d.angle_covariance*d.distance)**2+(d.angle*d.distance_covariance)**2
                cv_object.covariance = covariance
                cv_object_list.objects.append(cv_object)
            self.log_string += f'{len(cv_object_list.objects)} objects found, '
            cv_object_list.num_objects = len(cv_object_list.objects)
            self.object_publisher_.publish(cv_object_list)
                    
        self.get_logger().info(self.log_string)

    def image_listener_callback(self, data):
        '''
        Callback function when image from camera is recieved
        Calls detectObjects function using the image and the depth image
        Saves the image with boxes around objects in the node

        Parameters: 
            data - image from camera (imgmsg)
        '''
        self.get_logger().info('Camera Callback')
        self.header = data.header
        frame = self.br.imgmsg_to_cv2(data)
        self.cv_image, self.objects = self.object_detector.detectObjects(frame, self.depth_image)  

    def depth_listener_callback(self, data):
        '''
        Callback function when depth image from camera is recieved
        Saves image in the node
        Parameters: 
            data - depth image from camera (imgmsg)
        '''
        self.get_logger().info('Depth Callback') 
        self.depth_image = self.br.imgmsg_to_cv2(data)

def main(args=None):
    rclpy.init(args=args)
    
    object_detector_node = ObjectDetectorNode()
    
    rclpy.spin(object_detector_node)

    object_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
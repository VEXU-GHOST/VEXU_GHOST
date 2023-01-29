#!/usr/bin/env python3

from dis import dis
from turtle import color
import pyrealsense2 as rs
import numpy as np
import cv2
import imutils
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sqlalchemy import false, true
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from ghost_msgs.msg import CVDisc


class Disc:
    def __init__(this, x, y, radius):
        this.x = x
        this.y = y
        this.radius = radius

    def angle(this, hsize, hfov) -> float:
        angle = ((this.x - hsize/2)/(hsize/2))*(hfov/2)
        return angle

class DiscDetector:
    def __init__(this, hsize, vsize, hfov, vfov, lower_color, upper_color):
        this.hsize = hsize
        this.vsize = vsize
        this.hfov = hfov
        this.vfov = vfov
        this.lower_color = lower_color
        this.upper_color = upper_color

    def filterDisc(this, image):
        image = cv2.GaussianBlur(image, (11, 11), 0)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image = cv2.inRange(image, this.lower_color, this.upper_color)
        image = cv2.dilate(image, None, iterations=2)       
        image = cv2.erode(image, None, iterations=2)
        return image

    def findDiscs(this, image):
        contours = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        radius = 0

        discs = []
        for contour in contours:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > 10:
                discs.append(Disc(x, y, radius))
        return discs

    def addDiscToDisplay(this, image, disc: Disc, text: bool):
        meter_to_inch = 39.3701
        if disc.radius > 10:
            startpt = (int(disc.x) - int(disc.radius), int(disc.y) - int(disc.radius))
            endpt = (int(disc.x) + int(disc.radius), int(disc.y) + int(disc.radius))
            image = cv2.rectangle(image, startpt, endpt, (255, 255, 0), 1)
            if text:
                #distance = depth_frame.get_distance(int(disc.x),int(disc.y)) * meter_to_inch
                #basedist = math.sqrt(abs(distance*distance - 18*18))
                #hdist = round(np.sin(math.radians(disc.angle(this.hsize, this.hfov))) * distance, 3)
                angle = round(disc.angle(this.hsize,this.hfov),3)
                #text_dist = f'distance: {round(distance,3)}\"'
                #text_basedist = f'dist from base: {round(basedist,3)}\"'
                #text_hdist = f'hdistance: {hdist}\"'
                text_angle = f'angle: {angle}'
                #textList = [text_dist, text_hdist, text_angle, text_basedist]
                textList = [text_angle]
                for i in range(len(textList)):
                    text_position = (int(disc.x) + int(disc.radius), int(disc.y) + int(disc.radius) + i*15)
                    image = cv2.putText(image, textList[i], text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1, cv2.LINE_AA)
        return image
    
def detectDiscs(detector, color_image):
    try:
        starttime = time.perf_counter()

        disc_image = detector.filterDisc(color_image)
        discs = detector.findDiscs(disc_image)
        for d in discs:
            if not d:
                continue
            color_image = detector.addDiscToDisplay(color_image, d, true)
        
        fps = 1/(time.perf_counter() - starttime)
        color_image = cv2.putText(color_image, f"fps: {fps:.2f}", (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1, cv2.LINE_AA)
        return color_image, discs
    finally:
        a=0
    #return disc_image
    #detector.stopDisplay

def initCamera():
    hsize = 640
    vsize = 480
    hfov = 69
    vfov = 42
    #lower_color = (20,140,100)
    #upper_color = (50,255,255)
    lower_color = (50,140,100)
    upper_color = (100,255,255)
    camera_height = 18 #inch
    detector = DiscDetector(hsize, vsize, hfov, vfov, lower_color, upper_color)
    return detector

class DiscDetectorNode(Node):
    def __init__(self):
        super().__init__('disc_detector_node')
        self.subscription = self.create_subscription(
            #sensor_msgs/msg/Image,
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.image_publisher_ = self.create_publisher(Image, 'cv_frames', 10)
        self.disc_publisher_ = self.create_publisher(CVDisc, 'cv_discs', 10)
        self.detector = initCamera()
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.cv_image = None
        self.discs = []
        self.log_string = ''
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.log_string = ''
        try:
            if not self.cv_image:
                self.log_string += 'Video frame not found, '
        except:
            self.image_publisher_.publish(self.br.cv2_to_imgmsg(self.cv_image,encoding="rgb8"))
        
            # Display the message on the console
            self.log_string += 'Publishing video frame, '
        
        #try:
        if not self.discs:
            self.log_string += 'Discs not found, '
        else:
            #log_string += 'Discs found, '
            cv_disc = CVDisc()
            for i in range(len(self.discs)):
                cv_disc.disc_distance.append(1.0 / self.discs[i].radius)
                cv_disc.disc_direction.append(self.discs[i].angle(640,69))
                self.log_string += f'Disc {i} found, '
            self.disc_publisher_.publish(cv_disc)
                
                    
        self.get_logger().info(self.log_string)

    def listener_callback(self, data):
        self.get_logger().info('Camera Callback')
        frame = self.br.imgmsg_to_cv2(data)
        self.cv_image, self.discs = detectDiscs(self.detector, frame)   


def main(args=None):
    rclpy.init(args=args)
    
    disc_detector_node = DiscDetectorNode()
    
    rclpy.spin(disc_detector_node)

    disc_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

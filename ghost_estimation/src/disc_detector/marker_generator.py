#!/usr/bin/env python3

import numpy as np
import rclpy # Python library for ROS 2
#import math
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import PoseWithCovariance #, Pose
from ghost_msgs.msg import CVDiscList
from visualization_msgs.msg import Marker, MarkerArray

class DiscMarkerGeneratorNode(Node):
    def __init__(self):
        super().__init__('disc_detector_node')
        self.disc_subscription = self.create_subscription(
            #sensor_msgs/msg/Image,
            CVDiscList,
            '/cv_discs',
            self.disc_listener_callback,
            10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'cv_disc_markers', 10)
        self.log_string = ''
        self.header = None
        
    def disc_listener_callback(self, disc_list):
        self.get_logger().info('CVDiscList Callback')
        discs = disc_list.discs
        marker_array = MarkerArray()
        for disc in discs:
            marker = Marker()
            marker.header = disc_list.header
            marker.header.frame_id = 'camera_base_link'
            marker.ns = "cv_disc"
            marker.id = len(marker_array.markers)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.lifetime = rclpy.duration.Duration(seconds = 0.1).to_msg()
            marker.pose.position.x = disc.pose.position.x
            marker.pose.position.y = disc.pose.position.y
            marker.pose.position.z = disc.pose.position.z 
            marker.pose.orientation.x = disc.pose.orientation.x#0.0#0.707
            marker.pose.orientation.y = disc.pose.orientation.y#0.0
            marker.pose.orientation.z = disc.pose.orientation.z#0.0
            marker.pose.orientation.w = disc.pose.orientation.w#1.0#0.707
            marker.scale.x = disc.covariance[0]**.5
            marker.scale.y = disc.covariance[1+6*1]**.5
            marker.scale.z = 0.01
            marker.color.a = 0.5 # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
        

def main(args=None):
    rclpy.init(args=args)
    
    disc_marker_node = DiscMarkerGeneratorNode()
    
    rclpy.spin(disc_marker_node)

    disc_marker_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
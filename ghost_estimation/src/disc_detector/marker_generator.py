#!/usr/bin/env python3

import numpy as np
import rclpy # Python library for ROS 2
#import math
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import PoseWithCovariance #, Pose
from ghost_msgs.msg import CVObjList
from visualization_msgs.msg import Marker, MarkerArray

class MarkerGeneratorNode(Node):
    def __init__(self):
        super().__init__('marker_generator_node')
        self.obj_subscription = self.create_subscription(
            #sensor_msgs/msg/Image,
            CVObjList,
            '/cv_objs',
            self.obj_listener_callback,
            10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'cv_obj_markers', 10)
        self.log_string = ''
        self.header = None
        
    def obj_listener_callback(self, obj_list):
        self.get_logger().info('CVObjList Callback')
        objs = obj_list.objs
        marker_array = MarkerArray()
        for obj in objs:
            marker = Marker()
            marker.header = obj_list.header
            marker.header.frame_id = 'camera_base_link'
            marker.ns = "cv_obj"
            marker.id = len(marker_array.markers)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.lifetime = rclpy.duration.Duration(seconds = 0.1).to_msg()
            marker.pose = obj.pose
            marker.scale.x = obj.covariance[0]**.5
            marker.scale.y = obj.covariance[1+6*1]**.5
            marker.scale.z = 0.01
            marker.color.a = 0.5 # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
        

def main(args=None):
    rclpy.init(args=args)
    
    obj_marker_node = ObjMarkerGeneratorNode()
    
    rclpy.spin(obj_marker_node)

    obj_marker_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
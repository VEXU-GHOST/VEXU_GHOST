from math import pi
from typing import List

from requests import head

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


class PybulletRVIZPublisher():
    def __init__(self):
        rclpy.init()
        self.node = Node("pybullet_rviz_publisher")
        self.base_position_pub = self.node.create_publisher(TFMessage, "/tf", 10)
        self.base_vel_marker_pub = self.node.create_publisher(MarkerArray, "/base_vel_markers", 10)
        self.joint_state_pub = self.node.create_publisher(JointState, "/joint_states", 10)

        # Estimate max velocites to normalize arrow length
        self.max_linear_velocity = 2 # Max velocity at 2 m/s
        self.max_angular_velocity = 4*pi # max angular velocity at 2 rotations per second

        self.lin_vel_scale = 2.0
        self.ang_vel_scale = 2.0

    def update_base_position(self, base_position: List, base_orientation: List) -> None:
       
        # Publish world to base_link transform
        tf_base_stamped_msg = TransformStamped()
        
        # Set msg time stamp using ros time
        tf_base_stamped_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        tf_base_stamped_msg.header.frame_id = "world"
        tf_base_stamped_msg.child_frame_id = "base_link"

        tf_base_stamped_msg.transform.translation.x = base_position[0]
        tf_base_stamped_msg.transform.translation.y = base_position[1]
        tf_base_stamped_msg.transform.translation.z = base_position[2]

        tf_base_stamped_msg.transform.rotation.x = base_orientation[0]
        tf_base_stamped_msg.transform.rotation.y = base_orientation[1]
        tf_base_stamped_msg.transform.rotation.z = base_orientation[2]
        tf_base_stamped_msg.transform.rotation.w = base_orientation[3]

        tf_msg = TFMessage()
        tf_msg.transforms.append(tf_base_stamped_msg)

        self.base_position_pub.publish(tf_msg)

    def update_base_velocity(self, base_linear_velocity: List, base_angular_velocity: List) -> None:
        curr_ros_time = self.node.get_clock().now().to_msg()
        
        # Linear Velocity Marker
        lin_vel_arrow = Marker()
        lin_vel_arrow.header.frame_id = "base_link"
        lin_vel_arrow.header.stamp = curr_ros_time

        lin_vel_arrow.id = 0
        lin_vel_arrow.type = 0 # Arrow
        lin_vel_arrow.action = 0 # Add/Modify

        lin_vel_end_point = Point()
        lin_vel_end_point.x = base_linear_velocity[0]/self.max_linear_velocity * self.lin_vel_scale
        lin_vel_end_point.y = base_linear_velocity[1]/self.max_linear_velocity * self.lin_vel_scale
        
        lin_vel_arrow.points = [Point(), lin_vel_end_point]
        
        lin_vel_arrow.color.r = 1.0
        lin_vel_arrow.color.g = 0.0
        lin_vel_arrow.color.b = 0.0

        lin_vel_arrow.scale.x = 0.2
        lin_vel_arrow.scale.y = 0.3

        lin_vel_arrow.frame_locked = True

        # Angular Velocity Marker
        ang_vel_arrow = Marker()
        ang_vel_arrow.header.frame_id = "base_link"
        ang_vel_arrow.header.stamp = curr_ros_time

        ang_vel_arrow.id = 1
        ang_vel_arrow.type = 0 # Arrow
        ang_vel_arrow.action = 0 # Add/Modify

        ang_vel_end_point = Point()
        ang_vel_end_point.z = base_angular_velocity[2]/self.max_angular_velocity * self.ang_vel_scale
        
        ang_vel_arrow.points = [Point(), ang_vel_end_point]

        ang_vel_arrow.color.r = 0.0
        ang_vel_arrow.color.g = 0.0
        ang_vel_arrow.color.b = 1.0

        ang_vel_arrow.scale.x = 0.2
        ang_vel_arrow.scale.y = 0.3

        ang_vel_arrow.frame_locked = True

        marker_arr = MarkerArray()
        marker_arr.markers.append(lin_vel_arrow)
        marker_arr.markers.append(ang_vel_arrow)

        self.base_vel_marker_pub.publish(marker_arr)


    def update_joint_states(self, joint_states: List) -> None:
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        
        for i in range(len(joint_states)):
            msg.name.append(joint_states[i]["name"])
            msg.position.append(joint_states[i]["jointPosition"])
            msg.velocity.append(joint_states[i]["jointVelocity"])
        
        self.joint_state_pub.publish(msg)

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
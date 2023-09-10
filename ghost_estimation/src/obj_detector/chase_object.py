from rclpy.node import Node
import rclpy
from ghost_msgs.msg import CVObjList
from ghost_msgs.msg import V5MotorCommand


class ChaseObjectNode(Node):
    def __init__(self):
        super().__init__('chase_object_node')
        self.depth_subscription = self.create_subscription(
            CVObjList, 
            'cv_objects',                
            self.obj_listener_callback,                  
            10) 
        self.conrtol_publisher = self.create_publisher(
            V5MotorCommand, 
            'motor_commands', 
            10)
    def obj_listener_callback(self, msg):
        self.num_objs = msg.num_objs
        self.objs = msg.objs
        #if msg.num_objs > 0:
            
        # string motor_name
        # int32 device_id

        # int32 desired_angle
        # float32 desired_velocity
        # float32 desired_voltage

        # bool use_position_control
        # int32 current_limit
        # bool active

        # turn and go forward when angle is small enough

def main(args=None):
    rclpy.init(args=args)
    
    chase_object_node = ChaseObjectNode()
    
    rclpy.spin(chase_object_node)

    chase_object_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
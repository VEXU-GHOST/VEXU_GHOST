from launch import LaunchDescription
from launch_ros.actions import Node

# import os
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ghost_autonomy_share_dir = get_package_share_directory('ghost_autonomy')
    # bt_path = os.path.join(ghost_autonomy_share_dir, "config", "bt.xml")
    return LaunchDescription([
        Node(
            package='ghost_autonomy',
            executable='publish_test',
            # parameters=[{
            #     'bt_path': bt_path
            # }]
        )
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ilqr_node = Node(
        package='ghost_ilqr',
        executable='controller',
        parameters=[
            {'t_horizon': 5.0},
            {'del_t': 0.1},
        ],
        output='screen',
    )

    return LaunchDescription([
        ilqr_node
    ])
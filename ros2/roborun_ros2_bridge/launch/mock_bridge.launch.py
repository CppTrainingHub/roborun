from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roborun_ros2_bridge',
            executable='roborun_bridge_node',
            name='roborun_bridge',
            parameters=[{'backend': 'mock'}],
        ),
    ])

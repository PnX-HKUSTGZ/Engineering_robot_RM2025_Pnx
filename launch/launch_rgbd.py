from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='senserdrivers',
            executable='camera_driver',
            name='camera_driver',
        ),
        Node(
            package='senserdrivers',
            executable='mid360_driver',
            name='mid360_driver',
        )
    ])
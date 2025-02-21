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
            parameters=[
                {"ExposureTimeLower":15000},
                {"ExposureTimeUpper":15000},
                {"Gain":15},
            ]
        ),
        Node(
            package='calibratecamera',
            executable='calibrate_camera',
            name='calibrate_camera',
        )
    ])
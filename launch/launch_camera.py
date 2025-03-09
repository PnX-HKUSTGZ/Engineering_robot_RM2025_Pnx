from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    Path = {"Location":get_package_share_directory("interfaces")+"/../../../../"}

    return LaunchDescription([
        Node(
            package='senserdrivers',
            executable='camera_driver',
            name='camera_driver',
            parameters=[Path],
        )
    ])
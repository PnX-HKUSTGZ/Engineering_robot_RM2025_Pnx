from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
import launch
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rgbdgenerator',
            executable='rgbdgenerator',
            name='rgbdgenerator',
            parameters=[{"Location":"/home/lqx/code/Engineering_robot_RM2025_Pnx"}],
        ),
    ])
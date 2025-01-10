from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
import launch
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    configpath=os.path.join(get_package_share_directory('videodriver'),'config','cameraconfig.yaml')
    return LaunchDescription([
        Node(
            package='videodriver',
            executable='camera_driver',
            name='camera_driver',
            parameters=[configpath],
        ),
    ])
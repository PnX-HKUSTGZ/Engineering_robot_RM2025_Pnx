from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    config_file=os.path.join(
    get_package_share_directory('detect_arrow'),'config','ArrowDetectorConfig.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_file,description='Path to config file'),
        Node(
            package='detect_arrow',
            executable='detect_arrow',
            name='detect_arrow',
            output='screen',
            parameters=[config_file]
        )
    ])
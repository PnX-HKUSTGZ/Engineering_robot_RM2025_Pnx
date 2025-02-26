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
    '/home/lqx/code/Engineering_robot_RM2025_Pnx/src/c_strake_conmunication/config/serial_driver.yaml')
    print(config_file)
    return LaunchDescription([
        # DeclareLaunchArgument('config_file', default_value=config_file,description='Path to config file'),
        Node(
            package='c_strake_conmunication',
            executable='c_strake_conmunication',
            name='c_strake_conmunication',
            output='screen',
            parameters=[config_file]
        )
    ])
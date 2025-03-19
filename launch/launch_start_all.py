from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    Path = {"Location":get_package_share_directory("interfaces")+"/../../../../"}
    # Path = {"Location":"/home/pnx/code/Engineering_robot_RM2025_Pnx/"}
    return LaunchDescription([
        Node(
            package='senserdrivers',
            executable='camera_driver',
            name='camera_dirver',
            parameters=[Path],
            respawn=True,  # 核心参数：启用自动重启
            respawn_delay=3,  # 可选参数：延迟3秒重启
            output='screen'  # 输出到终端便于调试
        ),
        Node(
            package='senserdrivers',
            executable='mid360_driver',
            name='mid360_driver',
            parameters=[Path],
            respawn=True,  # 核心参数：启用自动重启
            respawn_delay=3,  # 可选参数：延迟3秒重启
            output='screen'  # 输出到终端便于调试
        ),
        Node(
            package='detect_arrow',
            executable='detect_arrow',
            name='detect_arrow',
            parameters=[Path],
            respawn=True,  # 核心参数：启用自动重启
            respawn_delay=3,  # 可选参数：延迟3秒重启
            output='screen'  # 输出到终端便于调试
        ),
        Node(
            package='c_strake_conmunication',
            executable='c_strake_conmunication',
            name='c_strake_conmunication',
            parameters=[Path],
            respawn=True,  # 核心参数：启用自动重启
            respawn_delay=3,  # 可选参数：延迟3秒重启
            output='screen'  # 输出到终端便于调试
        )
    ])
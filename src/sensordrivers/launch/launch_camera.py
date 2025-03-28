from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
import launch
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():

    Path = {"Location":get_package_share_directory("interfaces")+"/../../../../"}
    mid360_config_path = {"mid360_config_path":get_package_share_directory("sensordrivers")+"/config/mid360_config.json"}
    container = launch_ros.actions.ComposableNodeContainer(
        name="camera_driver",
        namespace="",
        package="rclcpp_components",
        executable='component_container_mt',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="sensordrivers",
                plugin="Engineering_robot_RM2025_Pnx::CameraDriver",
                name="camera_driver",
                parameters=[Path],
            ),
            launch_ros.descriptions.ComposableNode(
                package="sensordrivers",
                plugin="Engineering_robot_RM2025_Pnx::Mid360Driver",
                name="mid360_driver",
                parameters=[Path,mid360_config_path],
            )
        ],
        output="screen"
    )
    return launch.LaunchDescription([
        container,
    ])
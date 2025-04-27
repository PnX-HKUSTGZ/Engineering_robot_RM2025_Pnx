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
    SharedPath = get_package_share_directory("sensordrivers")
    realsenseConfig=os.path.join(SharedPath,"config","RealSenseconfig.yaml")

    print(f'realsenseConfig {realsenseConfig}')
    print(f'SharedPath {SharedPath}')
    camera_container = launch_ros.actions.ComposableNodeContainer(
        name="realsense_driver",
        namespace="",
        package="rclcpp_components",
        executable='component_container_mt',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="sensordrivers",
                plugin="Engineering_robot_RM2025_Pnx::RealSense",
                name="RealSenseDriver",
                parameters=[realsenseConfig],
            )
        ],
        output="screen"
    )
    return launch.LaunchDescription([
        camera_container,
    ])
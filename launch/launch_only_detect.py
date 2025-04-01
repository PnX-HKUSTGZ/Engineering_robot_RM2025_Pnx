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
    all = launch_ros.actions.ComposableNodeContainer(
        name="Engineering_robot_RM2025_Pnx_detect",
        namespace="",
        package="rclcpp_components",
        executable='component_container_mt',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package="target_redeem_box",
                plugin='Engineering_robot_RM2025_Pnx::RedeemBox_detector',
                name='RedeemBox_detector',
                parameters=[Path],
            )
        ],
        output="screen"
    )
    return LaunchDescription([
        all,
    ])
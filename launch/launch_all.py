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

    base_path = os.path.join(get_package_share_directory("interfaces"), os.pardir, os.pardir, os.pardir, os.pardir) # 示例相对路径
    Path = {"Location": base_path + "/"} # 添加斜杠以保持原始逻辑
    
    all = launch_ros.actions.ComposableNodeContainer(
        name="Engineering_robot_RM2025_Pnx",
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
                plugin="Engineering_robot_RM2025_Pnx::RealSense",
                name="realsense_driver",
                parameters=[Path],
            ),
            # launch_ros.descriptions.ComposableNode(
            #     package="target_redeem_box",
            #     plugin="Engineering_robot_RM2025_Pnx::RedeemBox_detector",
            #     name="target_redeem_box",
            #     parameters=[Path],
            # ),
        ],
        output="screen",
        respawn=True,
    )
    target_redeem_box=Node(
        package="target_redeem_box",
        executable="RedeemBox_detector_node",
        name="target_redeem_box",
        parameters=[Path],
        output="screen",
        respawn=True,
    )
    # mid360 = Node(
    #     package='sensordrivers',
    #     executable='mid360_driver',
    #     name='mid360_driver',
    #     output='screen',
    #     parameters=[Path]
    # )
    return LaunchDescription([
        all,
        target_redeem_box,
        # mid360,
    ])
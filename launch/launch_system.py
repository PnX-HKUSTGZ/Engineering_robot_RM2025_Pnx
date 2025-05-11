from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
import xacro
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch_ros.descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, LogInfo
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from srdfdom.srdf import SRDF
from launch.conditions import IfCondition

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("engineering_robot", package_name="engineering_robot_moveit").to_moveit_configs()
    robot_controllerPath = get_package_share_directory("engineering_robot_controller")
    engineering_robot_hardware_interfacePath = get_package_share_directory("engineering_robot_hardware_interface")
    config_path = os.path.join(robot_controllerPath,"config/sample_config.yaml")
    use_sim_time={"use_sim_time": True}
    real_controllers_config={"":os.path.join(engineering_robot_hardware_interfacePath,"engineering_robot_hardware_interface.xml")}
    ld = LaunchDescription()

    Path = {"Location":get_package_share_directory("interfaces")+"/../../../../"}
    ld.add_action(
        
        launch_ros.actions.ComposableNodeContainer(
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
            ],
            output="screen"
        )
    )

    ld.add_action(
        Node(
        package="target_redeem_box",
        executable="RedeemBox_detector_node",
        name="target_redeem_box",
        parameters=[Path],
        output="screen",
        respawn=True,
        )
    )
    # mid360 = Node(
    #     package='sensordrivers',
    #     executable='mid360_driver',
    #     name='mid360_driver',
    #     output='screen',
    #     parameters=[Path]
    # )


    launch_package_path = moveit_config.package_path

    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("use_fake_joint_driver", default_value=False))

    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[
                moveit_config.robot_description,
                # use_sim_time
                ],
            # namespace="example_robot",
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )
    # Fake joint driver
    if IfCondition(LaunchConfiguration("use_fake_joint_driver")):
        ld.add_action(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                ],
                remappings=[
                    ("/controller_manager/robot_description", "robot_description"),
                ],
            )
        )
    else:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    str(moveit_config.package_path / "config/ros2_controllers.yaml"),
                ],
                remappings=[
                    ("/controller_manager/robot_description", "robot_description"),
                ],
                output='screen',
            )
        )  

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    ld.add_action(
        Node(
        package="engineering_robot_controller",
        executable="engineering_robot_controller",
        name="engineering_robot_controller",
        parameters=[
            # config_path,
            # moveit_config.to_dict(),
            {"planning_plugins":["ompl_interface/OMPLPlanner", "pilz_industrial_motion_planner/CommandPlanner"]},
            use_sim_time
            ],
        )
    )

    return ld
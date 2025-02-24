from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    Path = {"Location":"/home/lqx/code/Engineering_robot_RM2025_Pnx"}
    return LaunchDescription([
        Node(
            package='senserdrivers',
            executable='camera_driver',
            name='camera_dirver',
            parameters=[Path],
        ),
        Node(
            package='detect_arrow',
            executable='detect_arrow',
            name='detect_arrow',
            parameters=[Path],
        )
    ])
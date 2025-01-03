from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='videodriver',
            executable='camera_dirver',
            name='camera_dirver',
            namespace='ENGINEER_RM_25',
            parameters=[
                {"ExposureTimeLower":15000},
                {"ExposureTimeUpper":15000},
                {"Gain":15},
                {"parampath","/home/lqx/code/Engineering_robot_RM2025_Pnx/cameraparam"}
            ]
        ),
        Node(
            package='detect_arrow',
            executable='detect_arrow',
            name='detect_arrow',
            namespace='ENGINEER_RM_25'
        )
    ])
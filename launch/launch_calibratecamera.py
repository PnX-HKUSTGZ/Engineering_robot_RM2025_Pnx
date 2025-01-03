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
        ),
        Node(
            package='calibratecamera',
            executable='calibrate_camera',
            name='calibrate_camera',
            namespace='ENGINEER_RM_25',
        )
    ])
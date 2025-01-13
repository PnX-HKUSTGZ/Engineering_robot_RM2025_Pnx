from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='videodriver',
            executable='video_driver',
            name='video_driver',
            namespace='ENGINEER_RM_25',
            parameters=[
                {'VideoPath':'/home/lqx/code/Engineering_robot_RM2025_Pnx/video/Video_20241228180155626.avi'},
                ],
        ),
    ])
from launch import LaunchDescription
from launch.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 启动节点并加载 YAML 配置文件中的参数
        Node(
            package='my_package',            # 包名
            executable='video_driver',            # 可执行文件名
            name='video_driver',             # 节点名称
            output='screen',                 # 输出到屏幕
            namespace='ENGINEER_RM_25',
            parameters=[LaunchConfiguration('config_file')],  # 引用配置文件
        ),
    ])
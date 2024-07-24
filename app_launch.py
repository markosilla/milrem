from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as pkg_dir
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udp',
            executable='udp_node',
            parameters=[os.path.join(pkg_dir('udp'), 'params/udp.yaml')],
            output='screen',
        ),
        Node(
            package='logger',
            executable='logger_node',
            parameters=[os.path.join(pkg_dir('logger'), 'params/logger.yaml')],
            output='screen',
        ),        
    ])

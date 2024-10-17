import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = os.path.join(
        get_package_share_directory('pointcloud_motion_capture'))

    config_file = os.path.join(pkg_share_dir, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='pointcloud_motion_capture',
            executable='point_cloud_reader',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='pointcloud_motion_capture',
            executable='motion_capture',
            output='screen',
        ),
    ])

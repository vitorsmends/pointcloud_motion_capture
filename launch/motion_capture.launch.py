from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='rs_pointcloud_launch.py',
            name='realsense2_camera',
            output='screen'
        ),
        Node(
            package='pointcloud_motion_capture',
            executable='point_cloud_reader',
            name='point_cloud_reader',
            output='screen'
        ),
        Node(
            package='pointcloud_motion_capture',
            executable='motion_capture',
            name='motion_capture',
            output='screen'
        )
    ])

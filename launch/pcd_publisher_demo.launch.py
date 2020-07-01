import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config_dir = os.path.join(get_package_share_directory(
        'pcd_publisher'), 'config', 'config.rviz')
    assert os.path.exists(rviz_config_dir)

    ply_path = os.path.join(get_package_share_directory(
        'pcd_publisher'), 'resource', 'teapot.ply')
    assert os.path.exists(rviz_config_dir)

    return LaunchDescription([
        Node(package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
        Node(package='pcd_publisher',
            node_executable='pcd_publisher_node',
            node_name='pcd_publisher_node',
            output='screen',
            arguments=[ply_path],
        ),
    ])


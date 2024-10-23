import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('lidar_view_sensor_proc'),
        'config.yaml')
    rviz = os.path.join(get_package_share_directory('lidar_view_sensor_proc'),
        'lidar_view.rviz')
    print(config)
    return LaunchDescription([
        Node(
            package="lidar_view_sensor_proc",
            # namespace="lidar_view_sensor_proc_node",
            executable="lidar_view_sensor_proc_node",
            output="screen",
            parameters=[config]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=['-d' + rviz]
        )
    ])
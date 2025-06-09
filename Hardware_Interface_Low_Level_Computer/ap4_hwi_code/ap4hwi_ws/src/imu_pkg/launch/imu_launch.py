from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():

    imu_node = launch_ros.actions.Node(
        package="imu_pkg",
        executable="imu_output_node",
        output="screen",
    )

    return LaunchDescription([imu_node])

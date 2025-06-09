from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="path_tracker",
                executable="path_recorder",
                name="path_recorder",
                output="screen",
                parameters=[{"robot_name": "AP4_digital_twin"}],
            )
        ]
    )

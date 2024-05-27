from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


# *
# Simple launch file to get xbox controller and feed forward logic working
# Created 17th of May 2023 for masters thesis at Infotiv spring 2023
#
# xbox controller is assumed to be passed through to docker container as js0
# this can be check with ls /dev/input/, js0 should show up
#
# *#
def generate_launch_description():
    # pkg_share = launch_ros.substitutions.FindPackageShare(package='xbox_controller_feed_forward_ctrl_pkg').find('xbox_controller_feed_forward_ctrl_pkg')

    # start joy node
    joy_node = feed_forward_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        # parameters=[{'autorepeat_rate': '5', 'coalesce_interval_ms' : '100'}]
    )

    teleop_twist_joy_node = launch_ros.actions.Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            {"enable_button": 5},
            {"require_enable_button": True},
            {"axis_linear.x": 4},
            {"axis_angular.yaw": 0},
        ],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=[("/cmd_vel", "/joystick_cmd_vel")],
    )
    # start teleop_twist_joy node, turn /joy into /cmd_vel topics
    # teleop_twist_joy_node = IncludeLaunchDescription(
    #        PythonLaunchDescriptionSource([get_package_share_directory('teleop_twist_joy'), '/launch/teleop-launch.py']),
    #                   launch_arguments={'joy_config': 'xbox'}.items()
    # )

    # feedforward controller
    feed_forward_node = Node(
        package="xbox_controller_feed_forward_ctrl_pkg",
        executable="feed_forward_node",
        output="screen",
    )

    return LaunchDescription([joy_node, teleop_twist_joy_node, feed_forward_node])

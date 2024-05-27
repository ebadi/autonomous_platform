from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


# *
#  AP4 Hardware Interface custom launch file
#
#   You can read more about ROS2 launch files here: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
#   WARNING: There is a difference between ROS(1) and ROS2 launch files, keep that in mind
#
#  This will be run each time the hardware interface docker container first starts up (or can be done manually by calling: ros2 launch launch_hwi_software_pkg launch_hwi_ros_software.launch.py )
#
#  The launch file starts up the following nodes
#       ros2_socketcan package to interface with CAN bus (2 of them)
#       can_ros2_interface - custom made ROS2 library to encode and decode data from can msgs
# *#
def generate_launch_description():
    return LaunchDescription(
        [
            # launch a launch file from ros2_socketcan, to start recieving can msgs from can0 interface and publish on ros topic /from_can_bus
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("ros2_socketcan"),
                        "/launch/socket_can_receiver.launch.py",
                    ]
                ),
                launch_arguments={"interface": "can0", "interval_sec": "0.01"}.items(),
            ),
            # launch a launch file from ros2_socketcan, to start sending can msgs to can0 interface and subscribe on ros topic /to_can_bus
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("ros2_socketcan"),
                        "/launch/socket_can_sender.launch.py",
                    ]
                ),
                launch_arguments={"interface": "can0", "timeout_sec": "0.01"}.items(),
            ),
            # launch custom made can translator node
            Node(package="can_msgs_to_ros2_topic_pkg", executable="can_ros2_interface"),
            # start feedforward controller and xbox joystick input nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory(
                            "xbox_controller_feed_forward_ctrl_pkg"
                        ),
                        "/launch/launch_xbox_controller_feedforward_ctrl.launch.py",
                    ]
                )
            ),
        ]
    )

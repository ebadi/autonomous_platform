import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="autonomous_platform_robot_description_pkg"
    ).find("autonomous_platform_robot_description_pkg")
    default_model_path = os.path.join(
        pkg_share, "src/description/ap4_robot_description.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro ", LaunchConfiguration("model")]),
                "use_sim_time": use_sim_time,
            }
        ],
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # include gazebo launch file, included in the gazebo_ros native pkg
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
        launch_arguments={
            "world_name": os.path.join(pkg_share, "world", "example_world.world")
        }.items(),
    )

    # run gazebo entity spawner node
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            #'-file', default_model_path,
            "-entity",
            "AP4_digital_twin",
            "-x",
            "2.0",
            "-y",
            "2.0",
            "-z",
            "0.5",
        ],
    )

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="gui",
                default_value="True",
                description="Flag to enable joint_state_publisher_gui",
            ),
            launch.actions.DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            launch.actions.DeclareLaunchArgument(
                name="use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            # joint_state_publisher_node, #This will be outputed from gazebo plugin
            # joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
            gazebo_node,
            spawn_entity_node,
        ]
    )

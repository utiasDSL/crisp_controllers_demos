import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")

    launch_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware",
        ),
        DeclareLaunchArgument(
            "left_robot_ip",
            default_value="172.16.1.2",
            description="IP adress of the left robot",
        ),
        DeclareLaunchArgument(
            "right_robot_ip",
            default_value="172.16.0.2",
            description="IP adress of the right robot",
        ),
    ]

    rviz_file = os.path.join(
        get_package_share_directory("crisp_controllers_robot_demos"),
        "config",
        "fr3",
        "dual_fr3.rviz",
    )
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("crisp_controllers_robot_demos"),
        "config",
        "fr3",
        "fr3_dual.urdf.xacro",
    )
    robot_description = xacro.process_file(franka_xacro_filepath).toprettyxml(indent="  ")

    left_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("crisp_controllers_robot_demos"),
                        "launch",
                        "franka.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "arm_id": "fr3",
            "arm_prefix": "left",
            "robot_ip": left_robot_ip,
            "use_rviz": "false",
            "use_fake_hardware": use_fake_hardware,
            "start_robot_state_publisher": "true"
        }.items(),
    )

    right_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("crisp_controllers_robot_demos"),
                        "launch",
                        "franka.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "arm_id": "fr3",
            "arm_prefix": "right",
            "robot_ip": right_robot_ip,
            "use_rviz": "false",
            "use_fake_hardware": use_fake_hardware,
            "start_robot_state_publisher": "true"
        }.items(),
    )

    # Nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_file],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
        ],
    )

    # Merge both joint states
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "left/joint_states",
                    "right/joint_states",
                ],
                "rate": 200,
            }
        ],
    )

    world_to_left_static_tf = Node(
        name="world_to_left_static_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "left_base"],
        output="screen",
    )
    world_to_right_static_tf = Node(
        name="world_to_right_static_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "1.0", "0", "0", "0", "0", "world", "right_base"],  # x, y, z, roll, pitch, yaw
        output="screen",
    )

    nodes = [
        joint_state_publisher,
        rviz_node,
        robot_state_publisher,
        world_to_left_static_tf,
        world_to_right_static_tf,
    ]

    return LaunchDescription(
        [
            *launch_arguments,
            GroupAction([
                PushRosNamespace("left"),
                left_ld,
            ]),
            GroupAction([
                PushRosNamespace("right"),
                right_ld,
            ]),
            *nodes,
        ]
    )

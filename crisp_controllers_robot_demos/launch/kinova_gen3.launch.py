# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of robot.",
            choices=["gen3", "gen3_lite"],
            default_value="gen3",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value='""',
            description="Name of the gripper attached to the arm",
        )
    )

    robot_type = LaunchConfiguration("robot_type")
    gripper = LaunchConfiguration("gripper")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("crisp_controllers_robot_demos"), "config", "kinova_gen3", "kinova_mujoco.urdf.xacro"]
            ),
            " ",
            "name:=kinova",
            " ",
            "arm:=",
            robot_type,
            " ",
            "gripper:=",
            gripper,
            " ",
            "use_fake_hardware:=True",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    kinova_controllers = PathJoinSubstitution([
        FindPackageShare("crisp_controllers_robot_demos"),
        "config",
        "kinova_gen3",
        "controllers.yaml",
    ])

    controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                kinova_controllers,
                robot_description,
            ],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
    )
    controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["pose_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["cartesian_impedance_controller", "--inactive"],
            output="screen",
        ),
    ]

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kortex_description"), "rviz", "view_robot.rviz"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        rviz,
        controller_manager,
        *controllers,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

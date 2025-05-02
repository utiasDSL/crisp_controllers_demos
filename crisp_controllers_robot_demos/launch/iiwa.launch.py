# Copyright 2022 ICube Laboratory, University of Strasbourg
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
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="iiwa_description",
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="iiwa_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="iiwa_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="iiwa.config.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="/",
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Start robot in Gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_planning",
            default_value="false",
            description="Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_servoing",
            default_value="false",
            description="Start robot with Moveit2 servoing.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="iiwa_arm_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.170.10.2",
            description="Robot IP of FRI interface",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="30200",
            description="Robot port of FRI interface.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="Configuration file of robot initial positions for simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "command_interface",
            default_value="position",
            description="Robot command interface [position|velocity|effort].",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "base_frame_file",
            default_value="base_frame.yaml",
            description="Configuration file of robot base frame wrt World.",
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_planning = LaunchConfiguration("use_planning")
    robot_controller = LaunchConfiguration("robot_controller")
    start_rviz = LaunchConfiguration("start_rviz")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    command_interface = LaunchConfiguration("command_interface")
    base_frame_file = LaunchConfiguration("base_frame_file")
    namespace = LaunchConfiguration("namespace")

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("crisp_controllers_robot_demos"),
            "config",
            "iiwa",
            "iiwa.urdf.xacro",
        ]),
        " ",
        "prefix:=",
        prefix,
        " ",
        "use_fake_hardware:=",
        use_fake_hardware,
        " ",
        "robot_ip:=",
        robot_ip,
        " ",
        "robot_port:=",
        robot_port,
        " ",
        "initial_positions_file:=",
        initial_positions_file,
        " ",
        "command_interface:=",
        command_interface,
        " ",
        "base_frame_file:=",
        base_frame_file,
        " ",
        "description_package:=",
        description_package,
        " ",
        "runtime_config_package:=",
        runtime_config_package,
        " ",
        "controllers_file:=",
        controllers_file,
        " ",
        "namespace:=",
        namespace,
    ])

    robot_description = {"robot_description": robot_description_content}

    # Use our controllers
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("crisp_controllers_robot_demos"),
        "config",
        "iiwa",
        "controllers.yaml",
    ])
    # robot_controllers = PathJoinSubstitution([
    #     FindPackageShare(runtime_config_package),
    #     "config",
    #     controllers_file,
    # ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(description_package),
        "rviz",
        "iiwa.rviz",
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        namespace=namespace,
        condition=UnlessCondition(use_sim),
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
        ],
        condition=UnlessCondition(use_planning),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            [namespace, "controller_manager"],
        ],
    )


    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_impedance_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
    )

    pose_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pose_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )


    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        cartesian_impedance_controller_spawner,
        pose_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)

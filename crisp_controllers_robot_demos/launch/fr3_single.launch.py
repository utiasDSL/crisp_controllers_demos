import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

import xacro


def robot_description_dependent_nodes_spawner(
    context: LaunchContext,
    robot_ip: LaunchConfiguration,
    arm_id: LaunchConfiguration,
    use_fake_hardware: LaunchConfiguration,
    namespace: LaunchConfiguration,
    start_robot_state_publisher: LaunchConfiguration,
):
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    namespace_str = context.perform_substitution(namespace)

    franka_xacro_filepath = os.path.join(
        get_package_share_directory("crisp_controllers_robot_demos"),
        "config",
        "fr3",
        "fr3_single.urdf.xacro",
    )

    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            "arm_id": arm_id_str,
            "robot_ip": robot_ip_str,
            "use_fake_hardware": use_fake_hardware_str,
            "arm_prefix": namespace_str,
        },
    ).toprettyxml(indent="  ")

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare("crisp_controllers_robot_demos"),
            "config",
            "controllers.yaml",
        ]
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                }
            ],
            condition=IfCondition(start_robot_state_publisher),
        ),
        # Instantiate the ros2_control node depending on the use_fake_hardware parameter
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                franka_controllers,
                {"robot_description": robot_description},
                {"arm_id": arm_id},
            ],
            remappings=[("joint_states", "franka/joint_states")],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            on_exit=Shutdown(),
            condition=UnlessCondition(use_fake_hardware),
        ),
    ]


def generate_launch_description():
    arm_id_parameter_name = "arm_id"
    robot_ip_parameter_name = "robot_ip"
    load_gripper_parameter_name = "load_gripper"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    namespace_name = "namespace"
    start_robot_state_publisher_name = "start_robot_state_publisher"

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    namespace = LaunchConfiguration(namespace_name, default="")
    start_robot_state_publisher = LaunchConfiguration(
        start_robot_state_publisher_name, default="true"
    )

    launch_functions = [
        OpaqueFunction(
            function=robot_description_dependent_nodes_spawner,
            args=[
                robot_ip,
                arm_id,
                use_fake_hardware,
                namespace,
                start_robot_state_publisher,
            ],
        )
    ]

    launch_arguments = [
        DeclareLaunchArgument(
            start_robot_state_publisher_name,
            description="Start the robot state publisher",
            default_value="true",
        ),
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description="Hostname or IP address of the robot.",
        ),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            description="ID of the type of arm used. Supported values: fer, fr3, fp3",
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value="false",
            description="Use fake hardware (i.e. start the robot in simulation)",
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value="false",
            description='Fake sensor commands. Only valid when "{}" is true'.format(
                use_fake_hardware_parameter_name
            ),
        ),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value="true",
            description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded without an end-effector.",
        ),
    ]

    nodes_that_require_namespace = [
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                {
                    "source_list": [
                        "franka/joint_states",
                        "franka_gripper/joint_states",
                    ],
                    "rate": 200,
                }
            ],
        ),
        *launch_functions,
        Node(
            package="controller_manager",
            executable="spawner",
            name="joint_state_broadcaster_spawner",
            arguments=[
                "joint_state_broadcaster",
                "-c",
                ["/", namespace, "/controller_manager"],
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="trajectory_controller_spawner",
            arguments=[
                "joint_trajectory_controller",
                "--inactive",
                "-c",
                ["/", namespace, "/controller_manager"],
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="gravity_compensation_spawner",
            arguments=[
                "gravity_compensation",
                "--inactive",
                "-c",
                ["/", namespace, "/controller_manager"],
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            name="cartesian_impedance_spawner",
            arguments=[
                "cartesian_impedance_controller",
                "--inactive",
                "-c",
                ["/", namespace, "/controller_manager"],
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(
        [
            *launch_arguments,
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    *nodes_that_require_namespace,
                ]
            ),
        ]
    )

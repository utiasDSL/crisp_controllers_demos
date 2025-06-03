"""Simple Node to allow users of crisp_py (https://github.com/utiasDSL/crisp_py) to use the Franka Hand (which we strongly discourage)."""

from time import time

import rclpy
from franka_msgs.action import Grasp, Homing
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GripperClient:
    def __init__(self, node: Node, gripper_namespace: str = "fr3_gripper"):
        """Initialize the gripper client."""

        self._node = node

        # The namespace for the gripper might change
        # check https://github.com/frankaemika/franka_ros2/issues/121
        self._grasp_client = ActionClient(
            node,
            Grasp,
            f"{gripper_namespace}/grasp",
            callback_group=ReentrantCallbackGroup(),
        )
        self._home_client = ActionClient(
            node,
            Homing,
            f"{gripper_namespace}/homing",
            callback_group=ReentrantCallbackGroup(),
        )
        self._gripper_state_subscriber = node.create_subscription(
            JointState,
            f"{gripper_namespace}/joint_states",
            self._gripper_state_callback,
            qos_profile_system_default,
        )
        self._width = None

    @property
    def width(self) -> float | None:
        """Returns the current width of the gripper or None if not initialized."""
        return self._width

    def is_open(self, open_threshold: float = 0.07) -> bool:
        """Returns True if the gripper is open."""
        return self.width > open_threshold

    def is_ready(self) -> bool:
        """Returns True if the gripper is fully ready to operate."""
        return self.width is not None

    def wait_until_ready(self, timeout_sec: float = 5.0):
        """Waits until the gripper is fully ready to operate."""
        time_start = time()
        while not self.is_ready():
            rclpy.spin_once(self._node, timeout_sec=1.0)
            if time() - time_start > timeout_sec:
                raise TimeoutError("Gripper client is not ready after timeout.")

    def _gripper_state_callback(self, msg: JointState):
        """Updates the gripper width using the current joint state."""
        self._width = msg.position[0] + msg.position[1]

    def home(self):
        """Homes the gripper."""
        goal = Homing.Goal()
        self._home_client.send_goal_async(goal)

    def grasp(
        self,
        width: float,
        speed: float = 0.1,
        force: float = 50.0,
        epsilon_outer: float = 0.08,
        epsilon_inner: float = 0.01,
        block: bool = False,
    ):
        """Grasp with the gripper and does not block.
        Args:
            width (float): The width of the gripper.
            speed (float, optional): The speed of the gripper. Defaults to 0.1.
            force (float, optional): The force of the gripper. Defaults to 50.0.
            epsilon_outer (float, optional): The outer epsilon of the gripper. Defaults to 0.08.
            epsilon_inner (float, optional): The inner epsilon of the gripper. Defaults to 0.01.
            block (bool, optional): Whether to block. Defaults to False.
        """
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.outer = epsilon_outer
        goal.epsilon.inner = epsilon_inner
        future = self._grasp_client.send_goal_async(
            goal
        )  # We assume that the server is running.

        if block:
            rate = self._node.create_rate(10)
            while not future.done():
                rate.sleep()
            goal_handle = future.result()
            future = goal_handle.get_result_async()

            while not future.done():
                rate.sleep()

            rate.destroy()

    def close(self, **grasp_kwargs):
        """Close the gripper.

        Args:
            **grasp_kwargs: Keyword arguments to pass to the grasp function. (check the grasp function for details)
        """
        self.grasp(width=0.0, **grasp_kwargs)

    def open(self, **grasp_kwargs):
        """Open the gripper.

        Args:
            **grasp_kwargs: Keyword arguments to pass to the grasp function. (check the grasp function for details)
        """
        self.grasp(width=0.08, **grasp_kwargs)

    def toggle(self, **grasp_kwargs):
        """Toggle the gripper between open and closed.

        Args:
            **grasp_kwargs: Keyword arguments to pass to the grasp function. (check the grasp function for details)
        """
        if self.is_open():
            self.close(**grasp_kwargs)
        else:
            self.open(**grasp_kwargs)


class CrispPyGripperAdapater(Node):
    def __init__(self):
        super().__init__("crisp_py_gripper_adapter")

        self.command_topic = "gripper/gripper_position_controller/commands"
        self.joint_state_topic = "gripper/joint_states"

        self.joint_state_freq = 50

        self.gripper_client = GripperClient(self)
        self.gripper_client.wait_until_ready()

        self.gripper_client.open()
        self.is_closing = False

        self.create_subscription(
            Float64MultiArray,
            self.command_topic,
            self.callback_command,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            self.joint_state_topic,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self.create_timer(1 / self.joint_state_freq, self.callback_publish_joint_state)
        self.get_logger().info("The crisp_py gripper adapter started.")

    def callback_publish_joint_state(self):
        if self.gripper_client.width is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = ["gripper_joint"]
        msg.position = [self.gripper_client.width / 0.08]
        msg.effort = [0.0]

        self.joint_state_publisher.publish(msg)

    def callback_command(self, msg: Float64MultiArray):
        """Callback to the gripper command."""
        # NOTE: this only allows to open and close. The FrankaHand is not super responsive anyway, this is just a
        # temporary node...
        gripper_command = msg.data[0]
        # self.get_logger().info(f"Received a command to move the gripper: {msg}")

        if (
            gripper_command <= 0.5
            and self.gripper_client.is_open()
            and not self.is_closing
        ):
            self.gripper_client.close()
            self.is_closing = True
        elif (
            gripper_command > 0.5
            and not self.gripper_client.is_open()
            and self.is_closing
        ):
            self.gripper_client.open()
            self.is_closing = False


def main():
    rclpy.init()
    adapter = CrispPyGripperAdapater()
    rclpy.spin(adapter)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

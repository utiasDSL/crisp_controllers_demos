"""Script to publish the target pose of the robot using an Rviz marker."""

from crisp_controllers_robot_demos.interactive_server import (
    create_interactive_marker_server,
)
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TargetPublisher(Node):
    """Interactive Marker Server to publish a target pose for the robot using an Rviz marker.
    This can be used to test algorithms that use the target pose."""

    def __init__(self):
        """Initialize the node.

        We initialize the transform buffer and the timer to publish the target pose.
        """
        super().__init__("target_publisher")

        prefix = self.get_namespace().strip("/")
        prefix_ = prefix + "_" if prefix else ""

        self._publish_frequency = self.declare_parameter(
            "publish_frequency",
            50.0,
            descriptor=ParameterDescriptor(
                description="The frequency to publish the pose."
            ),
        ).value
        self._target_frame = (
            prefix_
            + self.declare_parameter(
                "target_frame",
                "fr3_hand_tcp",
                descriptor=ParameterDescriptor(
                    description="The name of the frame interested in."
                ),
            ).value
        )
        self._base_frame = (
            prefix_
            + self.declare_parameter(
                "base_frame",
                "base",
                descriptor=ParameterDescriptor(description="The base frame."),
            ).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pose_stamped = None
        self.timer = self.create_timer(1.0 / self._publish_frequency, self.publish_pose)

        self.pose_publisher = self.create_publisher(
            PoseStamped, "target_pose", qos_profile=qos_profile_system_default
        )

        self.get_logger().info("Interactive marker server started.")

    def _get_latest_pose(self) -> PoseStamped:
        """Get the latest pose of the pose of interest."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self._base_frame, self._target_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {self._base_frame} to {self._target_frame}: {ex}",
                throttle_duration_sec=1.0,
            )
            return
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def process_feedback(self, feedback: PoseStamped):
        """Update the marker pose to the current target pose if the marker is moved."""
        self.target_pose_stamped = PoseStamped()
        self.target_pose_stamped.header = feedback.header
        self.target_pose_stamped.header.frame_id = self._base_frame
        self.target_pose_stamped.pose = feedback.pose

    def publish_pose(self):
        """Publish the marker pose as a target pose if it is not None."""
        if self.target_pose_stamped is None:
            current_pose = self._get_latest_pose()
            if current_pose is None:
                return
            create_interactive_marker_server(
                node=self,
                initial_pose=current_pose,
                feedback_callback=self.process_feedback,
                frame_id=self._base_frame,
            )
            self.target_pose_stamped = current_pose

        self.pose_publisher.publish(self.target_pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

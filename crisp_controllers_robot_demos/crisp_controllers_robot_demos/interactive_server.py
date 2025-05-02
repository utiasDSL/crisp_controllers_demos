"""Contains utility functions for creating interactive markers."""

import numpy as np
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


def create_interactive_marker_server(
    node: Node,
    frame_id: str,
    feedback_callback: callable,
    initial_pose: PoseStamped,
):
    """Create an interactive marker server and add a control to it.

    The interactive marker is a box which can be moved and rotated, it is used to set the target pose of the robot.
    This code is based on the interactive marker tutorial of rviz.
    http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started

    Args:
        node (Node): The node to create the interactive marker server.
        frame_id (str): The frame id of the interactive marker.
        feedback_callback (callable): The callback function to call when the interactive marker is moved.
        initial_pose (PoseStamped): The initial pose of the marker.
    """
    namespace = node.get_namespace() if node.get_namespace()!="/" else ""
    server = InteractiveMarkerServer(
        node=node,
        namespace=namespace
    )

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.name = "control_marker"
    int_marker.pose = initial_pose.pose

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.05
    box_marker.scale.y = 0.05
    box_marker.scale.z = 0.05
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    int_marker.controls.append(box_control)

    # === X ===
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_x"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    rotate_control.markers
    int_marker.controls.append(rotate_control)

    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "rotate_x"
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(rotate_control)

    # === Y ===
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_y"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    rotate_control.orientation.w = 1 / np.sqrt(2)
    rotate_control.orientation.y = 1 / np.sqrt(2)
    rotate_control.orientation.x = 0.0
    rotate_control.orientation.z = 0.0
    int_marker.controls.append(rotate_control)

    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "rotate_y"
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate_control.orientation.w = 1 / np.sqrt(2)
    rotate_control.orientation.y = 1 / np.sqrt(2)
    rotate_control.orientation.x = 0.0
    rotate_control.orientation.z = 0.0
    int_marker.controls.append(rotate_control)

    # === Z ===
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_z"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    rotate_control.orientation.w = 1 / np.sqrt(2)
    rotate_control.orientation.z = 1 / np.sqrt(2)
    rotate_control.orientation.x = 0.0
    rotate_control.orientation.y = 0.0
    int_marker.controls.append(rotate_control)

    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "rotate_z"
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate_control.orientation.w = 1 / np.sqrt(2)
    rotate_control.orientation.z = 1 / np.sqrt(2)
    rotate_control.orientation.x = 0.0
    rotate_control.orientation.y = 0.0
    int_marker.controls.append(rotate_control)

    server.insert(int_marker, feedback_callback=feedback_callback)
    server.applyChanges()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetFullCollisionBehavior


class CollisionBehaviorSetter(Node):
    def __init__(self):
        super().__init__("collision_behavior_setter")

        self.cli = self.create_client(
            SetFullCollisionBehavior, "/service_server/set_full_collision_behavior"
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.send_request()

    def send_request(self):
        req = SetFullCollisionBehavior.Request()
        self.get_logger().info("Sending request.")

        req.lower_torque_thresholds_nominal = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0]
        req.upper_torque_thresholds_nominal = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
        req.lower_torque_thresholds_acceleration = [
            25.0,
            25.0,
            22.0,
            20.0,
            19.0,
            17.0,
            14.0,
        ]
        req.upper_torque_thresholds_acceleration = [
            35.0,
            35.0,
            32.0,
            30.0,
            29.0,
            27.0,
            24.0,
        ]
        req.lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
        req.upper_force_thresholds_nominal = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]
        req.lower_force_thresholds_acceleration = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
        req.upper_force_thresholds_acceleration = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Collision behavior set successfully")
        else:
            self.get_logger().error("Failed to set collision behavior")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionBehaviorSetter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

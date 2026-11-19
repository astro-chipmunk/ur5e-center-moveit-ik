#!/usr/bin/env python3
"""Publish a frozen RViz start-state snapshot under a separate TF prefix."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class StartStateLatcher(Node):
    def __init__(self):
        super().__init__("start_state_latcher")
        self.held = None
        self.latest = None
        self.capture_requested = False
        self.publisher = self.create_publisher(JointState, "/start_state_joint_states", 10)
        self.live_sub = self.create_subscription(
            JointState, "/joint_states", self.on_live_state, 10
        )
        self.servo_target_sub = self.create_subscription(
            Float64, "/hand_servo_target", self.on_servo_target, 10
        )
        self.timer = self.create_timer(0.05, self.publish_held_state)

    @staticmethod
    def _copy_state(state):
        return dict(
            (name, float(state.position[i]))
            for i, name in enumerate(state.name)
            if i < len(state.position)
        )

    def on_live_state(self, msg):
        if msg.name and msg.position:
            self.latest = self._copy_state(msg)
            if self.capture_requested:
                self.held = self.latest.copy()
                self.capture_requested = False
                self.get_logger().info(
                    "Captured the pose before the hand servo move; ordinary MoveIt plans won't replace it."
                )

    def on_servo_target(self, _msg):
        if self.latest:
            self.held = self.latest.copy()
            self.get_logger().info(
                "Captured the current pose before a hand servo move."
            )
        else:
            self.capture_requested = True

    def publish_held_state(self):
        if not self.held:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.held.keys())
        msg.position = list(self.held.values())
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StartStateLatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

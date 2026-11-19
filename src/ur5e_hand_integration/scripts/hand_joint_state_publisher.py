#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def calculate_hand_state(servo):
    rev = {
        name: 0.0 for name in (
            "hand_rev1", "hand_rev2", "hand_rev3", "hand_rev4",
            "hand_rev7", "hand_rev8", "hand_rev9", "hand_rev10",
            "hand_rev11", "hand_rev12", "hand_rev13",
            "hand_rev14", "hand_rev15", "hand_rev16", "hand_rev17",
            "hand_rev20", "hand_rev21",
        )
    }

    servo_angle = servo + 72.0

    if servo_angle <= 162.0:
        rev["hand_rev1"] = 0.0
        rev["hand_rev2"] = 0.0
        rev12_deg = 90.0 - (servo_angle - 72.0)
        rev12 = -math.radians(rev12_deg)
        parallel = -rev12
        for name in (
            "hand_rev7", "hand_rev8", "hand_rev9",
            "hand_rev10", "hand_rev11",
        ):
            rev[name] = parallel
        rev["hand_rev12"] = rev12
        rev["hand_rev13"] = math.radians(servo_angle)
        rev["hand_rev3"] = parallel
        rev["hand_rev4"] = -parallel
    elif servo_angle <= 198.0:
        rev["hand_rev13"] = math.radians(servo_angle)
    else:
        theta_deg = max(0.0, min(90.0, (servo_angle - 198.0) * 13.0 / 12.0))
        theta = math.radians(theta_deg)
        rev["hand_rev1"] = -theta
        rev["hand_rev2"] = -theta
        rev["hand_rev3"] = theta
        rev["hand_rev4"] = -theta
        rev["hand_rev13"] = math.radians(servo_angle)
        rev["hand_rev14"] = theta / 13.0 * 12.0
        rev["hand_rev15"] = theta / 13.0 * 12.0
        rev["hand_rev16"] = theta
        rev["hand_rev17"] = theta
        rev["hand_rev20"] = theta
        rev["hand_rev21"] = -theta

    return rev


class HandJointStatePublisher(Node):
    """Keeps the displayed hand at its current angle until the arm is ready."""

    def __init__(self):
        super().__init__("hand_joint_state_publisher")
        self.publisher = self.create_publisher(JointState, "/hand_joint_states", 10)
        self.target_publisher = self.create_publisher(Float64, "/hand_servo_target", 10)
        self.command_subscription = self.create_subscription(
            Float64, "/hand_servo_command", self.on_servo_command, 10
        )
        self.declare_parameter("servo_deg", 0.0)
        self.current_servo_deg = 0.0
        self.last_target_sent = None
        self.timer = self.create_timer(0.05, self.publish_hand_state)

    @staticmethod
    def calculate_hand(servo):
        return calculate_hand_state(servo)

    def on_servo_command(self, msg):
        self.current_servo_deg = max(0.0, min(209.0, float(msg.data)))

    def publish_hand_state(self):
        target = max(0.0, min(209.0, float(self.get_parameter("servo_deg").value)))
        if self.last_target_sent is None or abs(target - self.last_target_sent) > 1e-9:
            target_msg = Float64()
            target_msg.data = target
            self.target_publisher.publish(target_msg)
            self.last_target_sent = target

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        state = calculate_hand_state(self.current_servo_deg)
        msg.name = list(state.keys())
        msg.position = list(state.values())
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

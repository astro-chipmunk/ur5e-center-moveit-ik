#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from hand_joint_state_publisher import calculate_hand_state


class JointStateCombiner(Node):

    def __init__(self):
        super().__init__("joint_state_combiner")

        self.arm_state = None
        self.hand_state = None

        self.arm_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.arm_callback,
            10
        )

        self.hand_sub = self.create_subscription(
            JointState,
            "/hand_joint_states",
            self.hand_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            "/combined_joint_states",
            10
        )
        self.move_group_publisher = self.create_publisher(
            JointState,
            "/move_group_joint_states",
            10
        )

        self.timer = self.create_timer(
            0.01,
            self.publish_combined
        )

    def arm_callback(self, msg):
        self.arm_state = msg

    def hand_callback(self, msg):
        self.hand_state = msg

    def publish_combined(self):

        if self.arm_state is None:
            return

        if self.hand_state is None:
            return

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        # -------------------------------------------------
        # Combine joint states while removing duplicate names.
        #
        # If the same joint exists in both messages,
        # the hand_state value takes priority.
        # -------------------------------------------------

        joint_positions = {}

        # First: arm state
        for i, name in enumerate(self.arm_state.name):
            if i < len(self.arm_state.position):
                joint_positions[name] = self.arm_state.position[i]

        # Second: hand state
        # Hand values overwrite duplicated names.
        for i, name in enumerate(self.hand_state.name):
            if i < len(self.hand_state.position):
                joint_positions[name] = self.hand_state.position[i]

        msg.name = list(joint_positions.keys())
        msg.position = list(joint_positions.values())

        self.publisher.publish(msg)

        # MoveIt's planning scene keeps the hand at servo 0. The live stream
        # above still drives robot_state_publisher and the compensator.
        planning_positions = {}
        for i, name in enumerate(self.arm_state.name):
            if i < len(self.arm_state.position):
                planning_positions[name] = self.arm_state.position[i]
        planning_positions.update(calculate_hand_state(0.0))
        planning_msg = JointState()
        planning_msg.header.stamp = msg.header.stamp
        planning_msg.name = list(planning_positions.keys())
        planning_msg.position = list(planning_positions.values())
        self.move_group_publisher.publish(planning_msg)


def main(args=None):

    rclpy.init(args=args)

    node = JointStateCombiner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

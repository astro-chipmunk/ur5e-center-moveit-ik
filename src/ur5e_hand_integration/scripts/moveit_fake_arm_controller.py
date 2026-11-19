#!/usr/bin/env python3
"""Lightweight FollowJointTrajectory server for the MoveIt/RViz mock simulation."""

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


class FakeArmTrajectoryController(Node):
    def __init__(self):
        super().__init__('moveit_fake_arm_controller')
        # Start away from the all-zero extended/singular posture.  That pose
        # makes RViz's flange pose targets unnecessarily hard for IK to solve.
        initial_positions = [0.2, -1.0, 1.2, -1.0, -1.2, 0.3]
        self.positions = dict(zip(JOINTS, initial_positions))
        self.publisher = self.create_publisher(JointState, '/raw_arm_joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_state)
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            '/ur5e_joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute,
            goal_callback=self.accept_goal,
            cancel_callback=self.cancel_goal,
        )
        self.get_logger().info('Mock arm trajectory action is ready for MoveIt.')

    @staticmethod
    def accept_goal(_request):
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_goal(_goal_handle):
        return CancelResponse.ACCEPT

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [self.positions[name] for name in JOINTS]
        self.publisher.publish(msg)

    def execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        if set(trajectory.joint_names) != set(JOINTS):
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = 'Trajectory must contain all six UR5e arm joints.'
            goal_handle.abort()
            return result

        indices = [trajectory.joint_names.index(name) for name in JOINTS]
        current = [self.positions[name] for name in JOINTS]
        previous_time = 0.0
        for point in trajectory.points:
            if len(point.positions) != len(JOINTS):
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Each trajectory point must contain six positions.'
                goal_handle.abort()
                return result
            target = [float(point.positions[i]) for i in indices]
            endpoint = float(point.time_from_start.sec) + point.time_from_start.nanosec * 1e-9
            duration = max(0.0, endpoint - previous_time)
            start = list(current)
            started = time.monotonic()
            while duration > 0.0:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = 'Trajectory was canceled.'
                    return result
                fraction = min(1.0, (time.monotonic() - started) / duration)
                # Cubic smoothstep starts and ends each mock motion at zero speed.
                smooth_fraction = fraction * fraction * (3.0 - 2.0 * fraction)
                current = [a + smooth_fraction * (b - a) for a, b in zip(start, target)]
                self.positions.update(dict(zip(JOINTS, current)))
                self.publish_state()
                if fraction >= 1.0:
                    break
                time.sleep(0.02)
            current = target
            self.positions.update(dict(zip(JOINTS, current)))
            self.publish_state()
            previous_time = endpoint

        # Keep publishing the settled state before reporting success. MoveIt
        # monitors /joint_states asynchronously and can otherwise display an
        # execution failure even though the mock arm visibly reached the goal.
        for _ in range(6):
            self.publish_state()
            time.sleep(0.05)

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = 'Trajectory completed in the mock controller.'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FakeArmTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

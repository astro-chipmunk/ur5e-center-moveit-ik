#!/usr/bin/env python3
"""Hold the live fingertip midpoint while the hand servo changes.

MoveIt's standard GetPositionIK service solves the arm IK. This node measures
the live midpoint and its transform from the flange from TF, converts the held
midpoint pose to a flange target, then asks MoveIt for the arm joint solution.
"""

import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def transform_matrix(transform):
    t = transform.translation
    q = transform.rotation
    xx, yy, zz, ww = q.x * q.x, q.y * q.y, q.z * q.z, q.w * q.w
    xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
    return [
        [ww + xx - yy - zz, 2 * (xy - wz), 2 * (xz + wy), t.x],
        [2 * (xy + wz), ww - xx + yy - zz, 2 * (yz - wx), t.y],
        [2 * (xz - wy), 2 * (yz + wx), ww - xx - yy + zz, t.z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def inverse_rigid(matrix):
    rotation = [[matrix[r][c] for c in range(3)] for r in range(3)]
    translation = [matrix[r][3] for r in range(3)]
    inverse = [[rotation[c][r] for c in range(3)] for r in range(3)]
    offset = [-sum(inverse[r][c] * translation[c] for c in range(3)) for r in range(3)]
    return [inverse[r] + [offset[r]] for r in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def multiply(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def matrix_to_quaternion(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s,
                0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return (0.25 * s, (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return ((m[0][1] + m[1][0]) / s, 0.25 * s,
                (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s,
            0.25 * s, (m[1][0] - m[0][1]) / s)


class MoveItCenterIK(Node):
    def __init__(self):
        super().__init__("moveit_center_ik")
        self.declare_parameter("group_name", "ur5e_arm")
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("ik_link", "flange")
        self.declare_parameter("flange_link", "flange")
        self.declare_parameter("hand_base_link", "hand_base_link")
        self.declare_parameter("left_finger_link", "hand_1st_link_left_1")
        self.declare_parameter("right_finger_link", "hand_1st_link_right_1")
        self.declare_parameter("tip_offset_m", -0.015)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("servo_command_topic", "/hand_servo_command")
        self.declare_parameter("servo_goal_topic", "/hand_servo_goal")
        self.declare_parameter("servo_step_deg", 0.25)
        self.declare_parameter(
            "trajectory_action",
            "/ur5e_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("max_joint_velocity_rad_s", 0.36)
        self.declare_parameter("minimum_trajectory_duration_sec", 0.05)
        self.declare_parameter("position_tolerance_m", 0.0005)
        self.declare_parameter("orientation_tolerance_rad", 0.005)
        self.declare_parameter("timeout_sec", 2.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_joint_state = None
        self.held_center = None
        self.busy = False
        self.last_attempt = 0.0
        self.last_servo_deg = None
        self.servo_goal_deg = None
        self.servo_step_pending_deg = None
        self.servo_publisher = self.create_publisher(
            Float64, self.get_parameter("servo_command_topic").value, 10)
        self.create_subscription(
            JointState, self.get_parameter("joint_state_topic").value,
            self.on_joint_state, 10)
        self.create_subscription(
            Float64, self.get_parameter("servo_command_topic").value,
            self.on_servo_command, 10)
        self.create_subscription(
            Float64, self.get_parameter("servo_goal_topic").value,
            self.on_servo_goal, 10)
        self.ik_client = self.create_client(
            GetPositionIK, self.get_parameter("ik_service").value)
        self.action_client = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter("trajectory_action").value)
        self.timer = self.create_timer(0.05, self.update_hold)
        self.get_logger().info(
            "Waiting for the initial fingertip-center pose; servo commands will "
            "be compensated using MoveIt's standard IK service.")

    def on_joint_state(self, msg):
        self.latest_joint_state = msg

    def on_servo_command(self, msg):
        self.last_servo_deg = max(0.0, min(209.0, float(msg.data)))
        self.get_logger().info(
            f"Servo command {self.last_servo_deg:.1f} deg; holding fingertip center.")

    def servo_angle_from_state(self):
        if self.latest_joint_state is None or "hand_rev13" not in self.latest_joint_state.name:
            return None
        index = self.latest_joint_state.name.index("hand_rev13")
        return max(0.0, min(209.0, math.degrees(self.latest_joint_state.position[index]) - 72.0))

    def on_servo_goal(self, msg):
        self.servo_goal_deg = max(0.0, min(209.0, float(msg.data)))
        current = self.servo_angle_from_state()
        if current is None:
            self.get_logger().warning("Servo goal received before the current hand angle is known.")
            self.servo_goal_deg = None
            return
        self.last_servo_deg = current
        self.servo_step_pending_deg = None
        self.get_logger().info(
            f"Stepping servo from {current:.1f} to "
            f"{self.servo_goal_deg:.1f} deg, in small fractional steps; "
            "each step waits for center compensation.")

    def current_transforms(self):
        frame = self.get_parameter("planning_frame").value
        flange = self.get_parameter("flange_link").value
        hand_base = self.get_parameter("hand_base_link").value
        left = self.get_parameter("left_finger_link").value
        right = self.get_parameter("right_finger_link").value
        tf_flange = self.tf_buffer.lookup_transform(frame, flange, Time())
        tf_hand = self.tf_buffer.lookup_transform(frame, hand_base, Time())
        tf_left = self.tf_buffer.lookup_transform(frame, left, Time())
        tf_right = self.tf_buffer.lookup_transform(frame, right, Time())

        offset = float(self.get_parameter("tip_offset_m").value)

        def tip_point(tf):
            t = tf.transform.translation
            q = tf.transform.rotation
            # Rotate the local fingertip offset by this finger link's TF pose.
            r = transform_matrix(tf.transform)
            p = [t.x + r[0][2] * offset,
                 t.y + r[1][2] * offset,
                 t.z + r[2][2] * offset]
            return p

        p_left, p_right = tip_point(tf_left), tip_point(tf_right)
        center = transform_matrix(tf_hand.transform)
        center[0][3] = 0.5 * (p_left[0] + p_right[0])
        center[1][3] = 0.5 * (p_left[1] + p_right[1])
        center[2][3] = 0.5 * (p_left[2] + p_right[2])
        return transform_matrix(tf_flange.transform), center

    def update_hold(self):
        if self.latest_joint_state is None:
            return
        try:
            flange, center = self.current_transforms()
        except Exception:
            return
        if self.held_center is None:
            self.held_center = center
            p = [center[i][3] for i in range(3)]
            self.get_logger().info(
                f"Holding initial fingertip center at ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}) m.")
            return
        if self.busy or self.get_clock().now().nanoseconds - self.last_attempt < 300_000_000:
            return

        actual_servo = self.servo_angle_from_state()
        if (self.servo_step_pending_deg is not None and actual_servo is not None
                and abs(actual_servo - self.servo_step_pending_deg) > 0.15):
            return
        if self.servo_step_pending_deg is not None and actual_servo is not None:
            self.servo_step_pending_deg = None
            self.last_servo_deg = actual_servo

        error = [self.held_center[i][3] - center[i][3] for i in range(3)]
        position_error = math.sqrt(sum(value * value for value in error))
        held_q = matrix_to_quaternion(self.held_center)
        current_q = matrix_to_quaternion(center)
        dot = abs(sum(a * b for a, b in zip(held_q, current_q)))
        orientation_error = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
        if (position_error <= float(self.get_parameter("position_tolerance_m").value)
                and orientation_error <= float(self.get_parameter("orientation_tolerance_rad").value)):
            self.advance_servo_if_ready()
            return

        # Current flange-to-center transform changes as the hand opens/closes.
        # Convert the held center target back to a flange pose, then let MoveIt
        # solve the six arm joints with the standard IK service.
        flange_to_center = multiply(inverse_rigid(flange), center)
        target_flange = multiply(self.held_center, inverse_rigid(flange_to_center))
        self.last_attempt = self.get_clock().now().nanoseconds
        self.busy = True
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.get_parameter("group_name").value
        ik.ik_link_name = self.get_parameter("ik_link").value
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = self.get_parameter("planning_frame").value
        ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        p = target_flange
        ik.pose_stamped.pose.position.x = p[0][3]
        ik.pose_stamped.pose.position.y = p[1][3]
        ik.pose_stamped.pose.position.z = p[2][3]
        q = matrix_to_quaternion(p)
        (ik.pose_stamped.pose.orientation.x,
         ik.pose_stamped.pose.orientation.y,
         ik.pose_stamped.pose.orientation.z,
         ik.pose_stamped.pose.orientation.w) = q
        ik.robot_state.joint_state = self.latest_joint_state
        ik.timeout.sec = int(float(self.get_parameter("timeout_sec").value))
        ik.timeout.nanosec = int((float(self.get_parameter("timeout_sec").value) % 1.0) * 1e9)
        ik.avoid_collisions = True
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.on_ik_result)

    def advance_servo_if_ready(self):
        if self.servo_goal_deg is None or self.last_servo_deg is None:
            return
        delta = self.servo_goal_deg - self.last_servo_deg
        if abs(delta) < 0.05:
            self.servo_goal_deg = None
            self.get_logger().info(f"Reached servo goal {self.last_servo_deg:.1f} deg.")
            return
        actual = self.servo_angle_from_state()
        if actual is None:
            return
        self.last_servo_deg = actual
        delta = self.servo_goal_deg - actual
        if abs(delta) < 0.05:
            self.servo_goal_deg = None
            self.get_logger().info(f"Reached servo goal {actual:.1f} deg.")
            return
        step = min(abs(delta), float(self.get_parameter("servo_step_deg").value))
        next_angle = actual + math.copysign(step, delta)
        msg = Float64()
        msg.data = next_angle
        self.servo_publisher.publish(msg)
        self.servo_step_pending_deg = next_angle
        self.get_logger().info(f"Servo step: {next_angle:.1f} deg")

    def on_ik_result(self, future):
        try:
            result = future.result()
            if result is None or result.error_code.val != result.error_code.SUCCESS:
                code = None if result is None else result.error_code.val
                self.get_logger().warning(f"MoveIt IK found no solution (error {code}).")
                self.busy = False
                return
            self.send_solution(result.solution.joint_state)
        except Exception as exc:
            self.get_logger().error(f"IK request failed: {exc}")
            self.busy = False

    def send_solution(self, solution):
        if not self.action_client.server_is_ready():
            self.get_logger().warning("Trajectory action server is not ready.")
            self.busy = False
            return
        positions = dict(zip(solution.name, solution.position))
        missing = [name for name in ARM_JOINTS if name not in positions]
        if missing:
            self.get_logger().error(f"IK solution missing arm joints: {missing}")
            self.busy = False
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(positions[name]) for name in ARM_JOINTS]
        current = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        max_delta = max(abs(float(positions[name]) - float(current[name])) for name in ARM_JOINTS)
        max_velocity = max(float(self.get_parameter("max_joint_velocity_rad_s").value), 1e-3)
        min_duration = max(float(self.get_parameter("minimum_trajectory_duration_sec").value), 0.01)
        duration = max(min_duration, max_delta / max_velocity)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning("Trajectory controller rejected IK result.")
            self.busy = False
            return
        future = goal_handle.get_result_async()
        future.add_done_callback(self.on_execution_result)

    def on_execution_result(self, future):
        result = future.result()
        if result is None or result.result.error_code != 0:
            code = None if result is None else result.result.error_code
            self.get_logger().warning(f"Arm trajectory failed (error {code}).")
        else:
            self.get_logger().info("MoveIt IK compensation step executed.")
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MoveItCenterIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

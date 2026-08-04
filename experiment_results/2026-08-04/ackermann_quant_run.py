#!/usr/bin/env python3
"""Run one fixed-duration Ackermann command and summarize ROS/Gazebo measurements."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


DATA_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def nearest_average(samples, start, end):
    selected = [sample for sample in samples if start <= sample[0] <= end]
    if not selected:
        raise RuntimeError(f"no samples in interval [{start:.3f}, {end:.3f}]")
    return np.mean(np.asarray([sample[1:] for sample in selected], dtype=float), axis=0)


def circle_fit(samples, start, end):
    points = np.asarray(
        [[sample[1], sample[2]] for sample in samples if start <= sample[0] <= end],
        dtype=float,
    )
    if len(points) < 20:
        raise RuntimeError(f"only {len(points)} points available for circle fit")
    matrix = np.column_stack((2.0 * points[:, 0], 2.0 * points[:, 1], np.ones(len(points))))
    rhs = np.sum(points * points, axis=1)
    a, b, c = np.linalg.lstsq(matrix, rhs, rcond=None)[0]
    radius = math.sqrt(max(0.0, c + a * a + b * b))
    residuals = np.sqrt((points[:, 0] - a) ** 2 + (points[:, 1] - b) ** 2) - radius
    return {
        "center_x_m": float(a),
        "center_y_m": float(b),
        "radius_m": float(radius),
        "residual_rms_m": float(np.sqrt(np.mean(residuals * residuals))),
        "samples": int(len(points)),
    }


def integrate_cmd(samples, start, end, field_index):
    selected = [(s[0], s[field_index]) for s in samples if start <= s[0] <= end]
    if len(selected) < 2:
        raise RuntimeError("insufficient /cmd_vel samples for integration")
    times = np.asarray([s[0] for s in selected], dtype=float)
    values = np.asarray([s[1] for s in selected], dtype=float)
    return float(np.trapz(values, times))


class QuantRun(Node):
    def __init__(self):
        super().__init__("ackermann_quant_run", parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.raw_pub = self.create_publisher(TwistStamped, "/cmd_vel_raw", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, DATA_QOS)
        self.create_subscription(TwistStamped, "/cmd_vel", self.on_cmd, DATA_QOS)
        self.create_subscription(JointState, "/joint_states", self.on_joints, DATA_QOS)
        self.create_subscription(TFMessage, "/ground_truth_poses", self.on_gt, DATA_QOS)
        self.gt = []
        self.odom = []
        self.cmd = []
        self.joints = []

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, msg):
        if not msg.transforms:
            return
        # Gazebo dynamic_pose/info orders the model pose first; the probe verifies
        # ackermann_car is the first entry for this world/model.
        transform = msg.transforms[0].transform
        self.gt.append(
            (
                self.now_s(),
                transform.translation.x,
                transform.translation.y,
                yaw_from_quaternion(transform.rotation),
            )
        )

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.odom.append(
            (
                self.now_s(),
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
        )

    def on_cmd(self, msg):
        self.cmd.append((self.now_s(), msg.twist.linear.x, msg.twist.angular.z))

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        left = positions.get("front_left_steering_joint")
        right = positions.get("front_right_steering_joint")
        if left is not None and right is not None:
            self.joints.append((self.now_s(), left, right))

    def publish(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.raw_pub.publish(msg)


def run(args):
    rclpy.init()
    node = QuantRun()
    wall_deadline = time.monotonic() + 40.0
    try:
        while (
            node.now_s() <= 0.0
            or not node.gt
            or not node.odom
            or node.raw_pub.get_subscription_count() < 1
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() > wall_deadline:
                raise RuntimeError("timed out waiting for /clock, GT, odom, or guard subscriber")

        settle_start = node.now_s()
        while node.now_s() < settle_start + 2.0:
            node.publish(0.0, 0.0)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)

        command_start = node.now_s()
        next_publish = command_start
        command_end = command_start + args.duration
        while node.now_s() < command_end:
            now = node.now_s()
            if now >= next_publish:
                node.publish(args.linear, args.angular)
                next_publish += 0.1
            rclpy.spin_once(node, timeout_sec=0.01)

        stop_end = command_end + 1.0
        next_publish = node.now_s()
        while node.now_s() < stop_end:
            now = node.now_s()
            if now >= next_publish:
                node.publish(0.0, 0.0)
                next_publish += 0.1
            rclpy.spin_once(node, timeout_sec=0.01)

        baseline_start = command_start - 0.25
        baseline_end = command_start
        final_start = stop_end - 0.25
        final_end = stop_end
        gt0 = nearest_average(node.gt, baseline_start, baseline_end)
        gt1 = nearest_average(node.gt, final_start, final_end)
        odom0 = nearest_average(node.odom, baseline_start, baseline_end)
        odom1 = nearest_average(node.odom, final_start, final_end)
        gt_dx = gt1[0] - gt0[0]
        gt_dy = gt1[1] - gt0[1]
        odom_dx = odom1[0] - odom0[0]
        odom_dy = odom1[1] - odom0[1]
        c0 = math.cos(gt0[2])
        s0 = math.sin(gt0[2])
        gt_longitudinal = c0 * gt_dx + s0 * gt_dy
        gt_lateral = -s0 * gt_dx + c0 * gt_dy

        result = {
            "kind": args.kind,
            "linear_input_mps": args.linear,
            "angular_input_radps": args.angular,
            "duration_sim_s": args.duration,
            "command_start_sim_s": command_start,
            "command_end_sim_s": command_end,
            "gt_distance_m": math.hypot(gt_dx, gt_dy),
            "gt_longitudinal_m": gt_longitudinal,
            "gt_lateral_m": gt_lateral,
            "odom_distance_m": math.hypot(odom_dx, odom_dy),
            "odom_gt_abs_error_m": abs(math.hypot(odom_dx, odom_dy) - math.hypot(gt_dx, gt_dy)),
            "odom_gt_rel_error_pct": (
                100.0 * abs(math.hypot(odom_dx, odom_dy) - math.hypot(gt_dx, gt_dy))
                / max(math.hypot(gt_dx, gt_dy), 1e-12)
            ),
            "cmd_linear_integral_m": integrate_cmd(node.cmd, command_start, stop_end, 1),
            "cmd_yaw_integral_rad": integrate_cmd(node.cmd, command_start, stop_end, 2),
            "sample_counts": {
                "ground_truth": len(node.gt),
                "odom": len(node.odom),
                "cmd_vel": len(node.cmd),
                "joint_states": len(node.joints),
            },
        }

        steady_start = command_start + min(0.75, args.duration * 0.2)
        steady_end = command_end - 0.1
        steady_cmd = [s for s in node.cmd if steady_start <= s[0] <= steady_end and abs(s[2]) > 1e-6]
        steady_joints = [s for s in node.joints if steady_start <= s[0] <= steady_end]
        if steady_joints:
            result["steering_mean_rad"] = {
                "front_left": float(np.mean([s[1] for s in steady_joints])),
                "front_right": float(np.mean([s[2] for s in steady_joints])),
            }
        if abs(args.angular) > 1e-9:
            result["gt_circle_fit"] = circle_fit(node.gt, steady_start, steady_end)
            result["odom_circle_fit"] = circle_fit(node.odom, steady_start, steady_end)
            if steady_cmd:
                ratios = [abs(s[1] / s[2]) for s in steady_cmd]
                result["cmd_reference_radius_m"] = float(np.median(ratios))
                result["cmd_steady_mean"] = {
                    "linear_mps": float(np.mean([s[1] for s in steady_cmd])),
                    "angular_radps": float(np.mean([s[2] for s in steady_cmd])),
                }

        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(result, sort_keys=True))
        return 0
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", required=True)
    parser.add_argument("--linear", type=float, required=True)
    parser.add_argument("--angular", type=float, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        return run(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

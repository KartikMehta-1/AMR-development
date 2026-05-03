#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalizationWaiter(Node):
    def __init__(self, args):
        super().__init__("amr_wait_for_localization")
        self.args = args
        self.start_time = time.monotonic()
        self.last_pose = None
        self.last_pose_wall_time = None

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            args.pose_topic,
            self.handle_pose,
            pose_qos,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def handle_pose(self, msg):
        self.last_pose = msg
        self.last_pose_wall_time = time.monotonic()

    def pose_is_fresh(self):
        if self.last_pose_wall_time is None:
            return False
        return (time.monotonic() - self.last_pose_wall_time) <= self.args.max_pose_age

    def lookup_tf(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.args.global_frame,
                self.args.odom_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return None

    def tf_is_fresh(self, transform):
        stamp = Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= self.args.max_tf_age

    def describe_pose(self):
        pose = self.last_pose.pose.pose
        return (
            f"x={pose.position.x:.3f}, "
            f"y={pose.position.y:.3f}, "
            f"yaw={yaw_from_quaternion(pose.orientation):.3f}"
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Wait for fresh AMCL localization output.")
    parser.add_argument("--pose-topic", default="/amcl_pose")
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--max-pose-age", type=float, default=3.0)
    parser.add_argument("--max-tf-age", type=float, default=3.0)
    parser.add_argument("--print-period", type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = LocalizationWaiter(args)
    deadline = time.monotonic() + args.timeout
    next_print = time.monotonic()

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            transform = node.lookup_tf()
            if node.pose_is_fresh() and transform is not None and node.tf_is_fresh(transform):
                print(f"AMCL localization ready: {node.describe_pose()}", flush=True)
                return 0

            if time.monotonic() >= next_print:
                pose_age = None
                if node.last_pose_wall_time is not None:
                    pose_age = time.monotonic() - node.last_pose_wall_time
                tf_state = "missing"
                if transform is not None:
                    stamp = Time.from_msg(transform.header.stamp)
                    tf_age = (node.get_clock().now() - stamp).nanoseconds / 1e9
                    tf_state = f"age={tf_age:.2f}s"
                pose_state = "missing" if pose_age is None else f"age={pose_age:.2f}s"
                print(f"Waiting for localization: amcl_pose {pose_state}, map->odom {tf_state}", flush=True)
                next_print = time.monotonic() + args.print_period

        print("Timed out waiting for fresh AMCL localization. Set RViz 2D Pose Estimate again.", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

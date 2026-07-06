#!/usr/bin/env python3

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


DEFAULT_POSES_FILE = (
    "/workspaces/AMR-development/ros_ws/src/amr_description/config/so101_named_poses.yaml"
)


class So101NamedPoseClient:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("so101_named_pose_client")
        self.action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            args.action_name,
        )
        self.current_positions = {}
        self.node.create_subscription(
            JointState,
            args.joint_states_topic,
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = float(position)

    def wait_until_ready(self):
        if not self.action_client.wait_for_server(timeout_sec=self.args.startup_timeout):
            raise RuntimeError(f"Action server not available: {self.args.action_name}")
        end = time.monotonic() + self.args.startup_timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.current_positions:
                return
        raise RuntimeError(f"No SO-101 samples on {self.args.joint_states_topic}")

    def send_pose(self, pose):
        names = [name for name in pose if name in self.current_positions]
        if not names:
            raise RuntimeError("Named pose has no joints found in current SO-101 state")

        max_delta = max(abs(float(pose[name]) - self.current_positions[name]) for name in names)
        steps = max(1, int(math.ceil(max_delta / self.args.max_step)))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names

        for step_index in range(steps + 1):
            alpha = step_index / float(steps)
            point = JointTrajectoryPoint()
            point.positions = [
                self.current_positions[name] + (float(pose[name]) - self.current_positions[name]) * alpha
                for name in names
            ]
            total_time = max(self.args.point_time, self.args.point_time * step_index)
            point.time_from_start.sec = int(total_time)
            point.time_from_start.nanosec = int((total_time - int(total_time)) * 1e9)
            goal.trajectory.points.append(point)

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Named pose goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(result.error_string)
        return names, steps

    def close(self):
        self.node.destroy_node()


def load_poses(path):
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data["so101"]["poses"]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("pose", nargs="?", help="Named pose to execute")
    parser.add_argument("--list", action="store_true", help="List available poses")
    parser.add_argument("--poses-file", default=DEFAULT_POSES_FILE)
    parser.add_argument(
        "--action-name",
        default="/so101_arm_controller/follow_joint_trajectory",
    )
    parser.add_argument("--joint-states-topic", default="/so101/joint_states")
    parser.add_argument("--max-step", type=float, default=0.08)
    parser.add_argument("--point-time", type=float, default=0.08)
    parser.add_argument("--startup-timeout", type=float, default=8.0)
    return parser.parse_args()


def main():
    args = parse_args()
    poses = load_poses(args.poses_file)
    if args.list:
        for name in sorted(poses):
            print(name)
        return
    if not args.pose:
        print("ERROR: provide a pose name or --list", file=sys.stderr)
        sys.exit(2)
    if args.pose not in poses:
        print(f"ERROR: unknown SO-101 pose '{args.pose}'", file=sys.stderr)
        print("Available poses:", ", ".join(sorted(poses)), file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    client = So101NamedPoseClient(args)
    try:
        client.wait_until_ready()
        names, steps = client.send_pose(poses[args.pose])
        print(f"Moved SO-101 to '{args.pose}' using {steps} interpolated steps")
        print("Joints:", ", ".join(names))
    finally:
        client.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

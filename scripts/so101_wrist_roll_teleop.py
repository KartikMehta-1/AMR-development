#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


HELP = """
SO-101 wrist-roll teleop
------------------------
a / left arrow   roll negative
d / right arrow  roll positive
s                hold current wrist roll
[ / ]            decrease / increase step
q or CTRL-C      quit

Only so101_wrist_roll is commanded. Shoulder, elbow, wrist flex, and gripper
are left untouched by this helper.
"""


JOINT_NAME = "so101_wrist_roll"


def save_terminal_settings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    if sys.platform == "win32" or settings is None:
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def get_key(settings):
    if sys.platform == "win32":
        return msvcrt.getwch()
    tty.setraw(sys.stdin.fileno())
    try:
        key = sys.stdin.read(1)
        if key == "\x1b":
            key += sys.stdin.read(2)
        return key
    finally:
        restore_terminal_settings(settings)


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


class WristRollTeleop:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("so101_wrist_roll_teleop")
        self.action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            args.action_name,
        )
        self.current_position = math.nan
        self.last_stamp = 0.0
        self.node.create_subscription(
            JointState,
            args.joint_states_topic,
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg):
        try:
            index = list(msg.name).index(JOINT_NAME)
        except ValueError:
            return
        self.current_position = float(msg.position[index])
        self.last_stamp = time.monotonic()

    def wait_until_ready(self):
        if not self.action_client.wait_for_server(timeout_sec=self.args.startup_timeout):
            raise RuntimeError(f"Action server not available: {self.args.action_name}")
        end = time.monotonic() + self.args.startup_timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if math.isfinite(self.current_position):
                return
        raise RuntimeError(f"No {JOINT_NAME} sample on {self.args.joint_states_topic}")

    def spin_for_fresh_position(self, timeout_sec=0.08):
        end = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def send_position(self, target):
        target = clamp(target, self.args.min_position, self.args.max_position)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [JOINT_NAME]

        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.args.point_time * 1e9)
        goal.trajectory.points.append(point)

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            print(f"failed: {result.error_string}")
            return False
        self.spin_for_fresh_position()
        print(f"{JOINT_NAME}: {self.current_position:+.3f} rad")
        return True

    def close(self):
        self.node.destroy_node()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--action-name",
        default="/so101_arm_controller/follow_joint_trajectory",
    )
    parser.add_argument("--joint-states-topic", default="/so101/joint_states")
    parser.add_argument("--step", type=float, default=0.04)
    parser.add_argument("--min-step", type=float, default=0.01)
    parser.add_argument("--max-step", type=float, default=0.10)
    parser.add_argument("--point-time", type=float, default=0.05)
    parser.add_argument("--min-position", type=float, default=-2.70)
    parser.add_argument("--max-position", type=float, default=2.80)
    parser.add_argument("--startup-timeout", type=float, default=8.0)
    return parser.parse_args()


def main():
    args = parse_args()
    settings = save_terminal_settings()

    rclpy.init()
    teleop = WristRollTeleop(args)
    step = clamp(args.step, args.min_step, args.max_step)

    try:
        teleop.wait_until_ready()
        print(HELP)
        print(f"current {JOINT_NAME}: {teleop.current_position:+.3f} rad")
        print(f"step: {step:.3f} rad")

        while rclpy.ok():
            key = get_key(settings)
            if key in ("q", "\x03"):
                break
            if key == "[":
                step = clamp(step * 0.75, args.min_step, args.max_step)
                print(f"step: {step:.3f} rad")
                continue
            if key == "]":
                step = clamp(step * 1.25, args.min_step, args.max_step)
                print(f"step: {step:.3f} rad")
                continue

            teleop.spin_for_fresh_position()
            if not math.isfinite(teleop.current_position):
                print("waiting for wrist state")
                continue

            if key in ("a", "\x1b[D"):
                teleop.send_position(teleop.current_position - step)
            elif key in ("d", "\x1b[C"):
                teleop.send_position(teleop.current_position + step)
            elif key == "s":
                teleop.send_position(teleop.current_position)

    finally:
        restore_terminal_settings(settings)
        teleop.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

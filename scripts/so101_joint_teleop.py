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


JOINT_LIMITS = {
    "so101_shoulder_pan": (-1.91986, 1.91986),
    "so101_shoulder_lift": (-1.74533, 1.74533),
    "so101_elbow_flex": (-1.69, 1.69),
    "so101_wrist_flex": (-1.65806, 1.65806),
    "so101_wrist_roll": (-2.74385, 2.84121),
    "so101_gripper": (-0.174533, 1.74533),
}

JOINT_MENU = [
    ("1", "so101_shoulder_pan"),
    ("2", "so101_shoulder_lift"),
    ("3", "so101_elbow_flex"),
    ("4", "so101_wrist_flex"),
    ("5", "so101_wrist_roll"),
    ("6", "so101_gripper"),
]


HELP = """
SO-101 joint teleop
-------------------
1 shoulder pan       4 wrist flex
2 shoulder lift      5 wrist roll
3 elbow flex         6 gripper

a / left arrow       selected joint negative
d / right arrow      selected joint positive
s                    hold selected joint
[ / ]                decrease / increase step
p                    print current joint states
h                    show this help
q or CTRL-C          quit

Only joints listed in --allowed-joints will be commanded. The bridge still
enforces its own allowed-joint gate as a second safety layer.
"""


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


def clamp_from_current(current, target, lower, upper):
    if current < lower:
        if target <= current:
            return current
        return min(target, upper)
    if current > upper:
        if target >= current:
            return current
        return max(target, lower)
    return clamp(target, lower, upper)


def parse_allowed_joints(value):
    return {item.strip() for item in value.split(",") if item.strip()}


class So101JointTeleop:
    def __init__(self, args):
        self.args = args
        self.allowed_joints = parse_allowed_joints(args.allowed_joints)
        self.current_positions = {}
        self.node = rclpy.create_node("so101_joint_teleop")
        self.action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            args.action_name,
        )
        self.node.create_subscription(
            JointState,
            args.joint_states_topic,
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_LIMITS:
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

    def spin_for_fresh_position(self, timeout_sec=0.08):
        end = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def print_state(self):
        self.spin_for_fresh_position()
        for _, name in JOINT_MENU:
            value = self.current_positions.get(name, math.nan)
            marker = "*" if name in self.allowed_joints else " "
            print(f"{marker} {name:20s} {value:+.3f} rad")

    def send_position(self, joint_name, target):
        if joint_name not in self.allowed_joints:
            print(f"{joint_name} is not enabled in this teleop session")
            return False
        lower, upper = JOINT_LIMITS[joint_name]
        current = self.current_positions.get(joint_name, math.nan)
        if not math.isfinite(current):
            print(f"waiting for {joint_name} state")
            return False
        target = clamp_from_current(current, target, lower, upper)
        if target == current:
            if current < lower:
                print(f"{joint_name}: below limit; press d/right to move back")
            elif current > upper:
                print(f"{joint_name}: above limit; press a/left to move back")
            else:
                print(f"{joint_name}: at limit")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint_name]
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
        actual = self.current_positions.get(joint_name, math.nan)
        print(f"{joint_name}: {actual:+.3f} rad")
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
    parser.add_argument(
        "--allowed-joints",
        default="so101_shoulder_pan,so101_shoulder_lift,so101_elbow_flex,so101_wrist_flex,so101_wrist_roll,so101_gripper",
    )
    parser.add_argument("--step", type=float, default=0.08)
    parser.add_argument("--min-step", type=float, default=0.01)
    parser.add_argument("--max-step", type=float, default=0.12)
    parser.add_argument("--point-time", type=float, default=0.05)
    parser.add_argument("--startup-timeout", type=float, default=8.0)
    return parser.parse_args()


def main():
    args = parse_args()
    settings = save_terminal_settings()
    selected_joint = "so101_wrist_roll"
    step = clamp(args.step, args.min_step, args.max_step)

    rclpy.init()
    teleop = So101JointTeleop(args)

    try:
        teleop.wait_until_ready()
        print(HELP)
        print(f"enabled joints: {', '.join(sorted(teleop.allowed_joints))}")
        print(f"selected: {selected_joint}, step: {step:.3f} rad")
        teleop.print_state()

        while rclpy.ok():
            key = get_key(settings)
            if key in ("q", "\x03"):
                break
            if key == "h":
                print(HELP)
                continue
            if key == "p":
                teleop.print_state()
                continue
            if key == "[":
                step = clamp(step * 0.75, args.min_step, args.max_step)
                print(f"step: {step:.3f} rad")
                continue
            if key == "]":
                step = clamp(step * 1.25, args.min_step, args.max_step)
                print(f"step: {step:.3f} rad")
                continue

            for menu_key, joint_name in JOINT_MENU:
                if key == menu_key:
                    selected_joint = joint_name
                    enabled = "enabled" if joint_name in teleop.allowed_joints else "blocked"
                    print(f"selected: {selected_joint} ({enabled})")
                    break
            else:
                teleop.spin_for_fresh_position()
                current = teleop.current_positions.get(selected_joint, math.nan)
                if not math.isfinite(current):
                    print(f"waiting for {selected_joint} state")
                    continue

                if key in ("a", "\x1b[D"):
                    teleop.send_position(selected_joint, current - step)
                elif key in ("d", "\x1b[C"):
                    teleop.send_position(selected_joint, current + step)
                elif key == "s":
                    teleop.send_position(selected_joint, current)

    finally:
        restore_terminal_settings(settings)
        teleop.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

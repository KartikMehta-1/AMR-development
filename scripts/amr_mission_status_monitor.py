#!/usr/bin/env python3

import argparse
import json
import sys
import time
from typing import Any, Dict, Optional

import rclpy
from amr_missions_msgs.msg import MissionStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


TERMINAL_STATES = {"succeeded", "error", "cancel_requested", "idle"}


def clear() -> None:
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def age_text(stamp: Optional[float], stale_after: float) -> str:
    if stamp is None:
        return "never"
    age = time.monotonic() - stamp
    stale = " STALE" if age > stale_after else ""
    return f"{age:.1f}s ago{stale}"


def speed_from_odom(msg: Odometry) -> float:
    vx = float(msg.twist.twist.linear.x)
    wz = float(msg.twist.twist.angular.z)
    return abs(vx) + abs(wz)


class MissionStatusMonitor(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("amr_mission_status_monitor")
        self.args = args
        self.last_render = 0.0
        self.status: Optional[MissionStatus] = None
        self.status_stamp: Optional[float] = None
        self.state_entered: Optional[float] = None
        self.previous_state: Optional[str] = None
        self.safety: Dict[str, Any] = {}
        self.safety_stamp: Optional[float] = None
        self.odom_speed: Optional[float] = None
        self.odom_stamp: Optional[float] = None

        qos = QoSProfile(depth=10)
        self.create_subscription(MissionStatus, args.mission_topic, self.status_cb, qos)
        self.create_subscription(String, args.safety_topic, self.safety_cb, qos)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, qos)

    def status_cb(self, msg: MissionStatus) -> None:
        now = time.monotonic()
        if msg.state != self.previous_state:
            self.state_entered = now
            self.previous_state = msg.state
        self.status = msg
        self.status_stamp = now

    def safety_cb(self, msg: String) -> None:
        self.safety_stamp = time.monotonic()
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.safety = data
        except json.JSONDecodeError:
            pass

    def odom_cb(self, msg: Odometry) -> None:
        self.odom_speed = speed_from_odom(msg)
        self.odom_stamp = time.monotonic()

    def elapsed_text(self) -> str:
        if self.state_entered is None:
            return "unknown"
        return f"{time.monotonic() - self.state_entered:.1f}s"

    def warning_text(self) -> str:
        if self.status is None:
            return "waiting for mission status"
        warnings = []
        state = self.status.state
        if state == "navigating":
            if self.state_entered is not None:
                elapsed = time.monotonic() - self.state_entered
                if elapsed > self.args.long_nav_sec:
                    warnings.append(f"navigating for {elapsed:.0f}s")
            if self.odom_speed is not None and self.odom_speed < self.args.idle_speed:
                warnings.append("robot odom is idle")
            ages = self.safety.get("ages_sec", {})
            if isinstance(ages, dict):
                amcl_age = ages.get("amcl")
                if amcl_age is not None and float(amcl_age) > self.args.amcl_stale_sec:
                    warnings.append(f"AMCL stale {float(amcl_age):.0f}s")
            if self.safety.get("healthy") is False:
                warnings.append("safety unhealthy")
        return ", ".join(warnings) if warnings else "none"

    def render(self) -> None:
        clear()
        print("== mission_status ==", flush=True)
        print("", flush=True)
        if self.status is None:
            print("state:        waiting", flush=True)
            print(f"updated:      {age_text(self.status_stamp, self.args.status_stale_sec)}", flush=True)
            return

        status = self.status
        print(
            f"state:        {status.state}    mission: {status.mission_type}    elapsed: {self.elapsed_text()}",
            flush=True,
        )
        print(f"updated:      {age_text(self.status_stamp, self.args.status_stale_sec)}", flush=True)
        print(f"target:       {', '.join(status.target_places) if status.target_places else 'none'}", flush=True)
        print(f"current:      {status.current_place or 'none'}", flush=True)
        print(f"loop/retry:   {status.current_loop}/{status.total_loops}    retries_left={status.retries_remaining}", flush=True)
        print(f"detail:       {status.detail}", flush=True)
        print("", flush=True)
        print(f"warning:      {self.warning_text()}", flush=True)
        print(
            f"odom speed:   {'unknown' if self.odom_speed is None else f'{self.odom_speed:.3f}'} "
            f"({age_text(self.odom_stamp, self.args.status_stale_sec)})",
            flush=True,
        )
        safety_state = self.safety.get("healthy")
        safety_mode = self.safety.get("mode", "unknown")
        print(
            f"safety:       {safety_state if safety_state is not None else 'unknown'} "
            f"mode={safety_mode} ({age_text(self.safety_stamp, self.args.status_stale_sec)})",
            flush=True,
        )

    def tick(self) -> None:
        now = time.monotonic()
        if now - self.last_render >= self.args.render_period:
            self.render()
            self.last_render = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Clean live AMR mission status monitor.")
    parser.add_argument("--mission-topic", default="/amr_missions/status")
    parser.add_argument("--safety-topic", default="/amr/safety_supervisor/status")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--render-period", type=float, default=0.5)
    parser.add_argument("--status-stale-sec", type=float, default=2.5)
    parser.add_argument("--long-nav-sec", type=float, default=20.0)
    parser.add_argument("--idle-speed", type=float, default=0.02)
    parser.add_argument("--amcl-stale-sec", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True, write_through=True)
    rclpy.init()
    node = MissionStatusMonitor(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

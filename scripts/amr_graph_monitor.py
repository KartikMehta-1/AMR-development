#!/usr/bin/env python3

import argparse
import sys
import time
from typing import Iterable, List, Sequence, Tuple

import rclpy
from rclpy.node import Node


CORE_PREFIXES = (
    "/amr",
    "/amr_stm",
    "/diff_drive_controller",
    "/controller_manager",
    "/bt_navigator",
    "/planner_server",
    "/controller_server",
    "/recoveries_server",
    "/waypoint_follower",
    "/map_server",
    "/amcl",
    "/lifecycle_manager",
    "/rviz",
)

CORE_TOPICS = {
    "/amcl_pose",
    "/clicked_point",
    "/cmd_vel",
    "/goal_pose",
    "/initialpose",
    "/joint_states",
    "/map",
    "/map_updates",
    "/odom",
    "/parameter_events",
    "/robot_description",
    "/rosout",
    "/scan",
    "/scan_filtered",
    "/tf",
    "/tf_static",
}

CORE_NODE_NAMES = {
    "/amr_firmware",
    "/amr_hardware",
    "/amr_link_watchdog",
    "/amr_mission_server",
    "/amr_safety_status_monitor",
    "/amr_safety_supervisor",
    "/amr_scan_sanitizer",
    "/amr_teleop_keyboard",
    "/amr_voice_asr",
    "/amr_voice_text_command",
    "/controller_manager",
    "/diff_drive_controller",
    "/joint_state_broadcaster",
    "/launch_ros_1",
    "/robot_state_publisher",
    "/rviz2",
    "/static_tf_pub_laser",
    "/ydlidar_ros2_driver_node",
}


def should_show_name(name: str, *, all_names: bool) -> bool:
    if all_names:
        return True
    if name in CORE_TOPICS:
        return True
    return any(name == prefix or name.startswith(f"{prefix}/") for prefix in CORE_PREFIXES)


def should_show_node(name: str, *, all_names: bool) -> bool:
    if all_names:
        return True
    if name in CORE_NODE_NAMES:
        return True
    return should_show_name(name, all_names=False)


def shorten_type(type_names: Sequence[str]) -> str:
    if not type_names:
        return "unknown"
    first = type_names[0]
    return first.rsplit("/", 1)[-1]


def clear() -> None:
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def print_rows(headers: Sequence[str], rows: Iterable[Sequence[object]], widths: Sequence[int]) -> None:
    fmt = "  ".join(f"{{:<{width}}}" for width in widths)
    print(fmt.format(*headers), flush=True)
    print(fmt.format(*["-" * min(width, len(header)) for width, header in zip(widths, headers)]), flush=True)
    for row in rows:
        values = [str(value) for value in row]
        clipped = [
            value if len(value) <= width else value[: max(0, width - 1)] + "."
            for value, width in zip(values, widths)
        ]
        print(fmt.format(*clipped), flush=True)


class GraphMonitor(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("amr_graph_monitor")
        self.args = args
        self.last_render = 0.0

    def topic_rows(self) -> List[Tuple[str, int, int, str]]:
        rows = []
        for topic, type_names in self.get_topic_names_and_types():
            if not should_show_name(topic, all_names=self.args.all):
                continue
            rows.append(
                (
                    topic,
                    self.count_publishers(topic),
                    self.count_subscribers(topic),
                    shorten_type(type_names),
                )
            )
        return sorted(rows, key=lambda row: row[0])[: self.args.limit]

    def node_rows(self) -> List[Tuple[str, int, int]]:
        rows = []
        for name, namespace in self.get_node_names_and_namespaces():
            full_name = f"/{name}" if namespace == "/" else f"{namespace.rstrip('/')}/{name}"
            if not should_show_node(full_name, all_names=self.args.all):
                continue
            try:
                pubs = len(self.get_publisher_names_and_types_by_node(name, namespace))
                subs = len(self.get_subscriber_names_and_types_by_node(name, namespace))
            except Exception:
                pubs = -1
                subs = -1
            rows.append((full_name, pubs, subs))
        return sorted(rows, key=lambda row: row[0])[: self.args.limit]

    def render_topics(self) -> None:
        rows = self.topic_rows()
        clear()
        print("== topics ==", flush=True)
        print(f"refresh: {self.args.period:.1f}s    shown: {len(rows)}    filter: {'all' if self.args.all else 'amr/core'}", flush=True)
        print("", flush=True)
        print_rows(("topic", "pub", "sub", "type"), rows, (44, 3, 3, 20))

    def render_nodes(self) -> None:
        rows = self.node_rows()
        clear()
        print("== nodes ==", flush=True)
        print(f"refresh: {self.args.period:.1f}s    shown: {len(rows)}    filter: {'all' if self.args.all else 'amr/core'}", flush=True)
        print("", flush=True)
        print_rows(("node", "pub", "sub"), rows, (52, 3, 3))

    def tick(self) -> None:
        now = time.monotonic()
        if now - self.last_render < self.args.period:
            return
        if self.args.mode == "topics":
            self.render_topics()
        else:
            self.render_nodes()
        self.last_render = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Lightweight ROS graph monitor for AMR tmux panes.")
    parser.add_argument("--mode", choices=("topics", "nodes"), required=True)
    parser.add_argument("--period", type=float, default=3.0, help="Refresh period in seconds.")
    parser.add_argument("--limit", type=int, default=80, help="Maximum rows to render.")
    parser.add_argument("--all", action="store_true", help="Show every ROS graph entry, not just AMR/core names.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True, write_through=True)
    rclpy.init()
    node = GraphMonitor(args)
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

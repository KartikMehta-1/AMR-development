#!/usr/bin/env python3

import argparse
import math
import sys
import time
from collections import deque
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float32, Int32, String, UInt32


CORE_PREFIXES = (
    "/amr",
    "/amr_stm",
    "/camera",
    "/controller_manager",
    "/diff_drive_controller",
    "/joint_state_broadcaster",
    "/scan",
    "/tf",
    "/tf_static",
    "/joint_states",
)

TOPIC_GROUPS = (
    ("STM state", ("/amr_stm/wheel_state", "/amr_stm/safety_state", "/amr_stm/fault_mask", "/amr_stm/comm_status", "/amr_stm/comm_ok", "/amr_stm/comm_fault", "/amr_stm/comm_fault_mask")),
    ("STM commands", ("/amr_stm/wheel_cmd_", "/amr_stm/duty_cmd_", "/amr_stm/enable", "/amr_stm/estop", "/amr_stm/clear_fault")),
    ("STM current", ("/amr_stm/current_",)),
    ("Drive", ("/diff_drive_controller/", "/joint_states", "/dynamic_joint_states")),
    ("Perception", ("/scan", "/camera/")),
    ("TF / system", ("/tf", "/tf_static", "/robot_description", "/parameter_events", "/rosout")),
)

NODE_GROUPS = (
    ("Robot HW", ("/amr_firmware", "/amr_hardware", "/amr_link_watchdog")),
    ("Control", ("/controller_manager", "/diff_drive_controller", "/joint_state_broadcaster")),
    ("Perception", ("/ydlidar_ros2_driver_node", "/camera/")),
    ("Description / TF", ("/robot_state_publisher", "/static_tf_pub_laser")),
    ("Launch / tools", ("/launch", "/amr_orin_dashboard_monitor", "/_ros2cli")),
)

STM_FAULTS = [
    (1 << 0, "ESTOP"),
    (1 << 1, "OC_LEFT"),
    (1 << 2, "OC_RIGHT"),
    (1 << 3, "STALL_LEFT"),
    (1 << 4, "STALL_RIGHT"),
    (1 << 5, "ENC_TIMEOUT_LEFT"),
    (1 << 6, "ENC_TIMEOUT_RIGHT"),
    (1 << 7, "ADC_STUCK"),
    (1 << 15, "GENERIC"),
]

COMM_FAULTS = [
    (1 << 0, "STARTUP_TIMEOUT_WAITING_FOR_WHEEL_STATE"),
    (1 << 1, "STALE_WHEEL_STATE"),
]

CTRL_STATES = {
    0: "INIT",
    1: "IDLE",
    2: "ENABLED",
    3: "FAULT",
}


def clear() -> None:
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def clip(value: Any, width: int) -> str:
    text = str(value)
    if len(text) <= width:
        return text
    return text[: max(0, width - 1)] + "."


def rows(headers: Sequence[str], data: Iterable[Sequence[Any]], widths: Sequence[int]) -> None:
    fmt = "  ".join(f"{{:<{width}}}" for width in widths)
    print(fmt.format(*headers), flush=True)
    print(fmt.format(*["-" * min(width, len(header)) for width, header in zip(widths, headers)]), flush=True)
    for row in data:
        print(fmt.format(*[clip(value, width) for value, width in zip(row, widths)]), flush=True)


def matches_any(name: str, patterns: Sequence[str]) -> bool:
    return any(name == pattern or name.startswith(pattern) for pattern in patterns)


def group_name(name: str, groups: Sequence[Tuple[str, Sequence[str]]]) -> str:
    for label, patterns in groups:
        if matches_any(name, patterns):
            return label
    return "Other"


def print_grouped(label_rows: Dict[str, List[Sequence[Any]]], widths: Sequence[int], headers: Sequence[str]) -> None:
    for label, _patterns in TOPIC_GROUPS:
        group_rows = label_rows.get(label, [])
        if not group_rows:
            continue
        print(f"-- {label} --", flush=True)
        rows(headers, group_rows, widths)
        print("", flush=True)
    other = label_rows.get("Other", [])
    if other:
        print("-- Other --", flush=True)
        rows(headers, other, widths)
        print("", flush=True)


def qos() -> QoSProfile:
    profile = QoSProfile(depth=20)
    profile.history = HistoryPolicy.KEEP_LAST
    profile.reliability = ReliabilityPolicy.BEST_EFFORT
    return profile


def age(stamp: Optional[float], stale_sec: float = 1.0) -> str:
    if stamp is None:
        return "never"
    value = time.monotonic() - stamp
    return f"{value:.2f}s" + (" STALE" if value > stale_sec else "")


def hz(stamps: Deque[float]) -> str:
    if len(stamps) < 2:
        return "waiting"
    dt = stamps[-1] - stamps[0]
    if dt <= 0:
        return "n/a"
    return f"{(len(stamps) - 1) / dt:.2f}"


def decode(mask: Optional[int], table: Sequence[Tuple[int, str]]) -> str:
    if mask is None:
        return "unknown"
    names = [name for bit, name in table if mask & bit]
    known = 0
    for bit, _name in table:
        known |= bit
    unknown = mask & ~known
    if unknown:
        names.append(f"UNKNOWN_0x{unknown:x}")
    return ", ".join(names) if names else "none"


class Dashboard(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("amr_orin_dashboard_monitor")
        self.args = args
        self.last_render = 0.0
        self.values: Dict[str, Any] = {}
        self.stamps: Dict[str, float] = {}
        self.wheel_names: List[str] = []
        self.wheel_pos: List[float] = []
        self.wheel_vel: List[float] = []
        self.topic_stamps: Dict[str, Deque[float]] = {
            "/amr_stm/wheel_state": deque(maxlen=60),
            "/scan": deque(maxlen=80),
        }

        q = qos()
        self.create_subscription(JointState, "/amr_stm/wheel_state", self.wheel_state_cb, q)
        self.create_subscription(Float32, "/amr_stm/wheel_cmd_left", self.store_cb("cmd_left"), q)
        self.create_subscription(Float32, "/amr_stm/wheel_cmd_right", self.store_cb("cmd_right"), q)
        self.create_subscription(Float32, "/amr_stm/duty_cmd_left", self.store_cb("duty_left"), q)
        self.create_subscription(Float32, "/amr_stm/duty_cmd_right", self.store_cb("duty_right"), q)
        self.create_subscription(Int32, "/amr_stm/current_left_ma", self.store_cb("current_left_ma"), q)
        self.create_subscription(Int32, "/amr_stm/current_right_ma", self.store_cb("current_right_ma"), q)
        self.create_subscription(UInt32, "/amr_stm/current_left_adc", self.store_cb("current_left_adc"), q)
        self.create_subscription(UInt32, "/amr_stm/current_right_adc", self.store_cb("current_right_adc"), q)
        self.create_subscription(UInt32, "/amr_stm/safety_state", self.store_cb("safety_state"), q)
        self.create_subscription(Int32, "/amr_stm/fault_mask", self.store_cb("fault_mask"), q)
        self.create_subscription(String, "/amr_stm/comm_status", self.store_cb("comm_status"), q)
        self.create_subscription(Bool, "/amr_stm/comm_ok", self.store_cb("comm_ok"), q)
        self.create_subscription(Bool, "/amr_stm/comm_fault", self.store_cb("comm_fault"), q)
        self.create_subscription(UInt32, "/amr_stm/comm_fault_mask", self.store_cb("comm_fault_mask"), q)
        self.create_subscription(LaserScan, "/scan", self.stamp_cb("/scan"), q)

    def store_cb(self, key: str):
        def callback(msg: Any) -> None:
            self.values[key] = getattr(msg, "data", msg)
            self.stamps[key] = time.monotonic()

        return callback

    def stamp_cb(self, topic: str):
        def callback(_msg: Any) -> None:
            self.topic_stamps[topic].append(time.monotonic())

        return callback

    def wheel_state_cb(self, msg: JointState) -> None:
        now = time.monotonic()
        self.wheel_names = list(msg.name)
        self.wheel_pos = list(msg.position)
        self.wheel_vel = list(msg.velocity)
        self.stamps["wheel_state"] = now
        self.topic_stamps["/amr_stm/wheel_state"].append(now)

    def wheel_value(self, side: str, values: Sequence[float]) -> Optional[float]:
        candidates = [f"{side}_wheel_joint", f"{side}_wheel"]
        for candidate in candidates:
            if candidate in self.wheel_names:
                idx = self.wheel_names.index(candidate)
                if idx < len(values):
                    return values[idx]
        idx = 0 if side == "left" else 1
        if idx < len(values):
            return values[idx]
        return None

    def core_topics(self) -> List[Tuple[str, int, int, str]]:
        topics = []
        for name, type_names in self.get_topic_names_and_types():
            if any(name == p or name.startswith(f"{p}/") for p in CORE_PREFIXES):
                topics.append((name, self.count_publishers(name), self.count_subscribers(name), type_names[0].rsplit("/", 1)[-1]))
        return sorted(topics)

    def core_nodes(self) -> List[Tuple[str, int, int]]:
        nodes = sorted(
            f"/{name}" if ns == "/" else f"{ns.rstrip('/')}/{name}"
            for name, ns in self.get_node_names_and_namespaces()
        )
        rows_out = []
        for full_name in nodes:
            name = full_name.rsplit("/", 1)[-1]
            namespace = full_name[: -len(name)].rstrip("/") or "/"
            try:
                pubs = len(self.get_publisher_names_and_types_by_node(name, namespace))
                subs = len(self.get_subscriber_names_and_types_by_node(name, namespace))
            except Exception:
                pubs = -1
                subs = -1
            rows_out.append((full_name, pubs, subs))
        return rows_out

    def render_topics(self) -> None:
        topics = self.core_topics()
        clear()
        print("== Topics ==", flush=True)
        print(f"shown={len(topics)} refresh={self.args.period:.1f}s", flush=True)
        print("", flush=True)

        grouped: Dict[str, List[Sequence[Any]]] = {}
        for topic in topics:
            grouped.setdefault(group_name(topic[0], TOPIC_GROUPS), []).append(topic)
        print_grouped(grouped, (31, 3, 3, 15), ("topic", "pub", "sub", "type"))

    def render_nodes(self) -> None:
        node_rows = self.core_nodes()
        clear()
        print("== Nodes ==", flush=True)
        print(f"shown={len(node_rows)} refresh={self.args.period:.1f}s", flush=True)
        print("", flush=True)

        grouped: Dict[str, List[Sequence[Any]]] = {}
        for row in node_rows:
            grouped.setdefault(group_name(row[0], NODE_GROUPS), []).append(row)

        for label, _patterns in NODE_GROUPS:
            group_rows = grouped.get(label, [])
            if not group_rows:
                continue
            print(f"-- {label} --", flush=True)
            rows(("node", "pub", "sub"), group_rows, (36, 3, 3))
            print("", flush=True)
        other = grouped.get("Other", [])
        if other:
            print("-- Other --", flush=True)
            rows(("node", "pub", "sub"), other, (36, 3, 3))
            print("", flush=True)

    def render_wheels(self) -> None:
        left_vel = self.wheel_value("left", self.wheel_vel)
        right_vel = self.wheel_value("right", self.wheel_vel)
        left_pos = self.wheel_value("left", self.wheel_pos)
        right_pos = self.wheel_value("right", self.wheel_pos)

        def rpm(value: Optional[float]) -> str:
            return "unknown" if value is None else f"{value * 60.0 / (2.0 * math.pi):.2f}"

        def rad(value: Optional[float]) -> str:
            return "unknown" if value is None else f"{value:.4f}"

        clear()
        print("== Wheels ==", flush=True)
        print(f"wheel_state age={age(self.stamps.get('wheel_state'))} hz={hz(self.topic_stamps['/amr_stm/wheel_state'])}", flush=True)
        print("", flush=True)
        data = [
            (
                "left",
                f"{float(self.values.get('cmd_left', 0.0)):.4f}",
                rad(left_vel),
                rpm(left_vel),
                f"{int(self.values.get('current_left_ma', 0))}",
                f"{float(self.values.get('duty_left', 0.0)):.4f}",
                rad(left_pos),
            ),
            (
                "right",
                f"{float(self.values.get('cmd_right', 0.0)):.4f}",
                rad(right_vel),
                rpm(right_vel),
                f"{int(self.values.get('current_right_ma', 0))}",
                f"{float(self.values.get('duty_right', 0.0)):.4f}",
                rad(right_pos),
            ),
        ]
        rows(("side", "cmd", "meas", "rpm", "mA", "duty"), [row[:6] for row in data], (6, 8, 8, 7, 6, 7))
        print(f"ages cmd L={age(self.stamps.get('cmd_left'))} R={age(self.stamps.get('cmd_right'))}", flush=True)

    def render_state(self) -> None:
        safety = self.values.get("safety_state")
        state = None if safety is None else (int(safety) >> 16) & 0xFFFF
        safety_fault = None if safety is None else int(safety) & 0xFFFF
        fault_mask = None if "fault_mask" not in self.values else int(self.values["fault_mask"])
        comm_fault_mask = None if "comm_fault_mask" not in self.values else int(self.values["comm_fault_mask"])

        clear()
        print("== AMR State / Safety ==", flush=True)
        print("", flush=True)
        print(f"comm_status:     {self.values.get('comm_status', 'unknown')} ({age(self.stamps.get('comm_status'), 2.0)})", flush=True)
        print(f"comm_ok:         {self.values.get('comm_ok', 'unknown')} ({age(self.stamps.get('comm_ok'), 2.0)})", flush=True)
        print(f"comm_fault:      {self.values.get('comm_fault', 'unknown')} ({age(self.stamps.get('comm_fault'), 2.0)})", flush=True)
        print(f"comm_fault_mask: {comm_fault_mask} [{decode(comm_fault_mask, COMM_FAULTS)}]", flush=True)
        print("", flush=True)
        print(f"safety_state:    {safety} ({age(self.stamps.get('safety_state'))})", flush=True)
        print(f"control_state:   {CTRL_STATES.get(state, f'UNKNOWN_{state}') if state is not None else 'unknown'}", flush=True)
        print(f"safety_faults:   {safety_fault} [{decode(safety_fault, STM_FAULTS)}]", flush=True)
        print(f"fault_mask:      {fault_mask} [{decode(fault_mask, STM_FAULTS)}] ({age(self.stamps.get('fault_mask'))})", flush=True)
        print("", flush=True)
        print(f"scan_hz:         {hz(self.topic_stamps['/scan'])}", flush=True)
        print(f"wheel_state_hz:  {hz(self.topic_stamps['/amr_stm/wheel_state'])}", flush=True)

    def tick(self) -> None:
        now = time.monotonic()
        if now - self.last_render < self.args.period:
            return
        if self.args.mode == "topics":
            self.render_topics()
        elif self.args.mode == "nodes":
            self.render_nodes()
        elif self.args.mode == "wheels":
            self.render_wheels()
        else:
            self.render_state()
        self.last_render = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("topics", "nodes", "wheels", "state"), required=True)
    parser.add_argument("--period", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True, write_through=True)
    rclpy.init()
    node = Dashboard(args)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3

import argparse
import json
import sys
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String, UInt32


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


def best_effort_qos() -> QoSProfile:
    qos = QoSProfile(depth=10)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


def decode_bits(mask: Optional[int], table) -> List[str]:
    if mask is None:
        return ["unknown"]
    names = [name for bit, name in table if mask & bit]
    known = 0
    for bit, _name in table:
        known |= bit
    unknown = mask & ~known
    if unknown:
        names.append(f"UNKNOWN_BITS_0x{unknown:x}")
    return names or ["none"]


def age_text(stamp: Optional[float], stale_after: float) -> str:
    if stamp is None:
        return "never"
    age = time.monotonic() - stamp
    stale = " STALE" if age > stale_after else ""
    return f"{age:.1f}s ago{stale}"


def value_text(value: Any, default: str = "unknown") -> str:
    if value is None:
        return default
    return str(value)


def list_text(values: Any, default: str = "none") -> str:
    if not values:
        return default
    if isinstance(values, list):
        return ", ".join(str(value) for value in values) or default
    return str(values)


def odom_speed_text(value: Any) -> str:
    if isinstance(value, dict):
        parts = []
        for key in ("linear_x", "angular_z"):
            if key in value and value[key] is not None:
                try:
                    parts.append(f"{key}={float(value[key]):.3f}")
                except (TypeError, ValueError):
                    parts.append(f"{key}={value[key]}")
        return ", ".join(parts) if parts else "unknown"
    if value is None:
        return "unknown"
    return str(value)


class SafetyStatusMonitor(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("amr_safety_status_monitor")
        self.args = args
        self.last_render = 0.0
        self.status: Dict[str, Any] = {}
        self.status_stamp: Optional[float] = None
        self.status_error: Optional[str] = None
        self.safety_state: Optional[int] = None
        self.safety_stamp: Optional[float] = None
        self.fault_mask: Optional[int] = None
        self.fault_stamp: Optional[float] = None
        self.comm_status: Optional[str] = None
        self.comm_stamp: Optional[float] = None
        self.comm_fault_mask: Optional[int] = None
        self.comm_fault_stamp: Optional[float] = None

        reliable = QoSProfile(depth=10)
        best_effort = best_effort_qos()
        self.create_subscription(String, args.supervisor_topic, self.status_cb, reliable)
        self.create_subscription(UInt32, f"{args.stm_ns}/safety_state", self.safety_cb, best_effort)
        self.create_subscription(Int32, f"{args.stm_ns}/fault_mask", self.fault_cb, best_effort)
        self.create_subscription(String, f"{args.stm_ns}/comm_status", self.comm_status_cb, reliable)
        self.create_subscription(UInt32, f"{args.stm_ns}/comm_fault_mask", self.comm_fault_cb, reliable)

    def status_cb(self, msg: String) -> None:
        self.status_stamp = time.monotonic()
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self.status = data
                self.status_error = None
            else:
                self.status_error = "status JSON was not an object"
        except json.JSONDecodeError as exc:
            self.status_error = f"invalid status JSON: {exc}"

    def safety_cb(self, msg: UInt32) -> None:
        self.safety_state = int(msg.data)
        self.safety_stamp = time.monotonic()

    def fault_cb(self, msg: Int32) -> None:
        self.fault_mask = int(msg.data)
        self.fault_stamp = time.monotonic()

    def comm_status_cb(self, msg: String) -> None:
        self.comm_status = msg.data
        self.comm_stamp = time.monotonic()

    def comm_fault_cb(self, msg: UInt32) -> None:
        self.comm_fault_mask = int(msg.data)
        self.comm_fault_stamp = time.monotonic()

    def control_state(self) -> str:
        if self.safety_state is None:
            status_safety = self.status.get("safety_state", {})
            state = status_safety.get("control_state") if isinstance(status_safety, dict) else None
        else:
            state = (self.safety_state >> 16) & 0xFFFF
        if state is None:
            return "unknown"
        return f"{CTRL_STATES.get(int(state), f'UNKNOWN_{state}')} ({state})"

    def safety_fault_mask(self) -> Optional[int]:
        if self.safety_state is not None:
            return self.safety_state & 0xFFFF
        status_safety = self.status.get("safety_state", {})
        if isinstance(status_safety, dict):
            value = status_safety.get("fault_mask")
            return None if value is None else int(value)
        return None

    def health_label(self) -> str:
        if self.status.get("healthy") is True:
            return "OK"
        if self.status.get("healthy") is False:
            return "UNSAFE"
        return "waiting"

    def stale_names(self) -> List[str]:
        stale = self.status.get("stale", {})
        if not isinstance(stale, dict):
            return []
        return sorted(name for name, value in stale.items() if value)

    def render(self) -> None:
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

        print("== safety_status ==", flush=True)
        print("", flush=True)
        print(
            f"health:       {self.health_label()}    "
            f"mode: {value_text(self.status.get('mode'))}    "
            f"updated: {age_text(self.status_stamp, self.args.status_stale_sec)}",
            flush=True,
        )
        print(
            f"intervention: {value_text(self.status.get('intervention_active'))}    "
            f"count: {value_text(self.status.get('intervention_count'), '0')}",
            flush=True,
        )
        print(f"reasons:      {list_text(self.status.get('intervention_reasons'))}", flush=True)
        observed = self.status.get("observed_reasons")
        if observed:
            print(f"observed:     {list_text(observed)}", flush=True)
        print("", flush=True)

        fault_mask = self.fault_mask if self.fault_mask is not None else self.status.get("fault_mask")
        comm_fault_mask = (
            self.comm_fault_mask
            if self.comm_fault_mask is not None
            else self.status.get("comm_fault_mask")
        )
        comm_status = self.comm_status if self.comm_status is not None else self.status.get("comm_status")
        safety_fault_mask = self.safety_fault_mask()
        print(
            f"stm control:  {self.control_state()}    "
            f"safety: {value_text(self.safety_state)} ({age_text(self.safety_stamp, self.args.topic_stale_sec)})",
            flush=True,
        )
        print(
            f"stm fault:    {value_text(fault_mask)} "
            f"[{', '.join(decode_bits(None if fault_mask is None else int(fault_mask), STM_FAULTS))}] "
            f"({age_text(self.fault_stamp, self.args.topic_stale_sec)})",
            flush=True,
        )
        print(
            f"safety bits:  {value_text(safety_fault_mask)} "
            f"[{', '.join(decode_bits(safety_fault_mask, STM_FAULTS))}]",
            flush=True,
        )
        print(
            f"comm:         {value_text(comm_status)} ({age_text(self.comm_stamp, self.args.comm_stale_sec)})",
            flush=True,
        )
        print(
            f"comm fault:   {value_text(comm_fault_mask)} "
            f"[{', '.join(decode_bits(None if comm_fault_mask is None else int(comm_fault_mask), COMM_FAULTS))}] "
            f"({age_text(self.comm_fault_stamp, self.args.comm_stale_sec)})",
            flush=True,
        )
        print("", flush=True)

        stale = self.stale_names()
        print(f"stale:        {', '.join(stale) if stale else 'none'}", flush=True)
        ages = self.status.get("ages_sec", {})
        if isinstance(ages, dict):
            compact = []
            for name in ("stm", "comm", "odom", "scan", "amcl"):
                if name in ages:
                    compact.append(f"{name}={value_text(ages.get(name))}s")
            if compact:
                print(f"ages:         {', '.join(compact)}", flush=True)
        print(f"odom speed:   {odom_speed_text(self.status.get('odom_speed'))}", flush=True)
        if self.status_error:
            print("", flush=True)
            print(f"status error: {self.status_error}", flush=True)

    def tick(self) -> None:
        now = time.monotonic()
        if now - self.last_render >= self.args.render_period:
            self.render()
            self.last_render = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Clean live AMR safety status monitor.")
    parser.add_argument("--supervisor-topic", default="/amr/safety_supervisor/status")
    parser.add_argument("--stm-ns", default="/amr_stm")
    parser.add_argument("--render-period", type=float, default=0.25)
    parser.add_argument("--topic-stale-sec", type=float, default=1.0)
    parser.add_argument("--comm-stale-sec", type=float, default=2.0)
    parser.add_argument("--status-stale-sec", type=float, default=2.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True, write_through=True)
    rclpy.init()
    node = SafetyStatusMonitor(args)
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

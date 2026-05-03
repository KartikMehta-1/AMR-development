#!/usr/bin/env python3

import argparse
import json
import sys
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Int32, String, UInt32
from std_srvs.srv import Trigger


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


def best_effort_qos():
    qos = QoSProfile(depth=10)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


def reliable_qos():
    return QoSProfile(depth=10)


def decode_bits(mask, table):
    names = [name for bit, name in table if mask & bit]
    known = 0
    for bit, _name in table:
        known |= bit
    unknown = mask & ~known
    if unknown:
        names.append(f"UNKNOWN_BITS_0x{unknown:x}")
    return names or ["none"]


def yes_no(prompt, default=False):
    suffix = " [y/N]: " if not default else " [Y/n]: "
    try:
        answer = input(prompt + suffix).strip().lower()
    except EOFError:
        print("")
        return default
    if not answer:
        return default
    return answer in ("y", "yes")


class SafetyRecover(Node):
    def __init__(self, args):
        super().__init__("amr_safety_recover")
        self.args = args
        self.last = {}

        be = best_effort_qos()
        rel = reliable_qos()
        self.create_subscription(Int32, "/amr_stm/fault_mask", self._store("fault_mask"), be)
        self.create_subscription(UInt32, "/amr_stm/safety_state", self._store("safety_state"), be)
        self.create_subscription(String, "/amr_stm/comm_status", self._store("comm_status"), rel)
        self.create_subscription(UInt32, "/amr_stm/comm_fault_mask", self._store("comm_fault_mask"), rel)
        self.create_subscription(String, "/amr/safety_supervisor/status", self._store("supervisor_status"), rel)

        self.cmd_pub = self.create_publisher(Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)
        self.enable_pub = self.create_publisher(Bool, "/amr_stm/enable", 10)
        self.clear_pub = self.create_publisher(Empty, "/amr_stm/clear_fault", 10)

        self.mission_cancel = self.create_client(Trigger, "/amr_missions/cancel")
        self.nav_cancel = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.reset_client = self.create_client(Trigger, "/amr/safety_supervisor/reset_intervention")

    def _store(self, name):
        def callback(msg):
            self.last[name] = getattr(msg, "data", None)

        return callback

    def spin_for(self, duration_sec):
        end = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_topics(self):
        needed = {"fault_mask", "safety_state", "comm_status", "comm_fault_mask"}
        end = time.monotonic() + self.args.topic_timeout
        while rclpy.ok() and time.monotonic() < end:
            if needed.issubset(self.last):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = sorted(needed - set(self.last))
        print(f"ERROR: timed out waiting for topics: {', '.join(missing)}")
        return False

    def print_state(self, label):
        fault_mask = int(self.last.get("fault_mask", 0))
        safety_state = int(self.last.get("safety_state", 0))
        comm_fault_mask = int(self.last.get("comm_fault_mask", 0))
        control_state = (safety_state >> 16) & 0xFFFF
        safety_fault_mask = safety_state & 0xFFFF

        print(f"\n== {label} ==")
        print(f"fault_mask: {fault_mask} ({', '.join(decode_bits(fault_mask, STM_FAULTS))})")
        print(
            "safety_state: %s control=%s fault_bits=%s"
            % (
                safety_state,
                CTRL_STATES.get(control_state, f"UNKNOWN_{control_state}"),
                ",".join(decode_bits(safety_fault_mask, STM_FAULTS)),
            )
        )
        print(f"comm_status: {self.last.get('comm_status', 'unknown')}")
        print(f"comm_fault_mask: {comm_fault_mask} ({', '.join(decode_bits(comm_fault_mask, COMM_FAULTS))})")

        status_raw = self.last.get("supervisor_status")
        if status_raw:
            try:
                status = json.loads(status_raw)
                print(
                    "supervisor: healthy=%s intervention_active=%s reasons=%s"
                    % (
                        status.get("healthy"),
                        status.get("intervention_active"),
                        status.get("intervention_reasons") or status.get("last_intervention_reasons"),
                    )
                )
            except json.JSONDecodeError:
                print("supervisor: status present but not valid JSON")
        else:
            print("supervisor: no status sample captured")

    def call_trigger(self, client, name, timeout_sec):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            print(f"WARN: service unavailable: {name}")
            return None
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            print(f"WARN: service timed out: {name}")
            return None
        return future.result()

    def cancel_nav2(self):
        if not self.nav_cancel.wait_for_service(timeout_sec=self.args.service_timeout):
            print("WARN: Nav2 cancel service unavailable")
            return
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        future = self.nav_cancel.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.service_timeout)
        if future.done():
            print("Nav2 cancel requested.")
        else:
            print("WARN: Nav2 cancel request timed out.")

    def publish_zero(self):
        self.cmd_pub.publish(Twist())
        self.spin_for(0.2)
        print("Published zero cmd_vel.")

    def set_enable(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.enable_pub.publish(msg)
        self.spin_for(0.2)
        print(f"Published /amr_stm/enable={str(enabled).lower()}.")

    def clear_fault(self):
        self.clear_pub.publish(Empty())
        self.spin_for(0.2)
        print("Published /amr_stm/clear_fault.")

    def wait_for_fault_clear(self):
        end = time.monotonic() + self.args.clear_timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if int(self.last.get("fault_mask", -1)) == 0:
                return True
        return False

    def run(self):
        if not self.wait_for_topics():
            return 2
        self.print_state("captured fault state")

        print("\nStopping autonomy and motor output...")
        response = self.call_trigger(self.mission_cancel, "/amr_missions/cancel", self.args.service_timeout)
        if response is not None:
            print(f"Mission cancel: success={response.success} message='{response.message}'")
        self.cancel_nav2()
        self.publish_zero()
        self.set_enable(False)

        self.spin_for(0.5)
        self.print_state("after stop/disable")

        fault_mask = int(self.last.get("fault_mask", 0))
        if fault_mask != 0:
            print("\nA latched STM fault is active. Fix the physical cause before clearing it.")
            if not self.args.assume_fixed:
                if not yes_no("Physical cause fixed and safe to clear STM fault?", default=False):
                    print("Leaving STM disabled. Fault was not cleared.")
                    return 1
        else:
            print("\nNo active STM fault mask; continuing to supervisor reset.")

        if fault_mask != 0 or self.args.clear_when_zero:
            self.clear_fault()
            if not self.wait_for_fault_clear():
                self.print_state("clear failed")
                print("ERROR: fault_mask did not clear. Leaving STM disabled.")
                return 1

        self.spin_for(0.5)
        self.print_state("after STM fault clear")

        response = self.call_trigger(
            self.reset_client,
            "/amr/safety_supervisor/reset_intervention",
            self.args.service_timeout,
        )
        if response is None:
            print("ERROR: supervisor reset service did not respond. Leaving STM disabled.")
            return 1
        print(f"Supervisor reset: success={response.success} message='{response.message}'")
        if not response.success:
            print("ERROR: supervisor still sees an unsafe state. Leaving STM disabled.")
            return 1

        self.spin_for(0.5)
        self.print_state("after supervisor reset")

        should_enable = self.args.reenable
        if self.args.prompt_reenable:
            should_enable = yes_no("Re-enable STM motor output now?", default=False)
        if should_enable:
            self.set_enable(True)
            self.spin_for(1.0)
            self.print_state("final")
        else:
            print("Leaving STM disabled. Re-enable manually when ready:")
            print('  ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: true}"')

        return 0


def parse_args():
    parser = argparse.ArgumentParser(description="Guarded AMR safety recovery helper.")
    parser.add_argument("--assume-fixed", action="store_true", help="Do not prompt before clearing a nonzero STM fault.")
    parser.add_argument("--reenable", action="store_true", help="Re-enable STM after supervisor reset without prompting.")
    parser.add_argument("--no-prompt-reenable", dest="prompt_reenable", action="store_false", help="Do not ask about re-enable; leave STM disabled unless --reenable is set.")
    parser.add_argument("--clear-when-zero", action="store_true", help="Publish clear_fault even if fault_mask is already zero.")
    parser.add_argument("--topic-timeout", type=float, default=5.0)
    parser.add_argument("--service-timeout", type=float, default=3.0)
    parser.add_argument("--clear-timeout", type=float, default=5.0)
    parser.set_defaults(prompt_reenable=True)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = SafetyRecover(args)
    try:
        return node.run()
    except KeyboardInterrupt:
        print("\nInterrupted. Leaving STM in its current state.")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

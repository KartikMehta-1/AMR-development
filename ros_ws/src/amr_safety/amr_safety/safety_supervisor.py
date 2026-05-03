#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
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
    return names


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("amr_safety_supervisor")

        self.declare_parameter("monitor_only", True)
        self.declare_parameter("enforce", False)
        self.declare_parameter("auto_reenable_when_safe", False)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("amcl_topic", "/amcl_pose")
        self.declare_parameter("require_amcl", False)
        self.declare_parameter("max_stm_age_sec", 0.5)
        self.declare_parameter("max_comm_age_sec", 1.5)
        self.declare_parameter("max_odom_age_sec", 0.5)
        self.declare_parameter("max_scan_age_sec", 0.5)
        self.declare_parameter("max_amcl_age_sec", 2.0)
        self.declare_parameter("publish_period_sec", 1.0)

        self.monitor_only = bool(self.get_parameter("monitor_only").value)
        self.enforce = bool(self.get_parameter("enforce").value)
        self.auto_reenable_when_safe = bool(self.get_parameter("auto_reenable_when_safe").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.amcl_topic = str(self.get_parameter("amcl_topic").value)
        self.require_amcl = bool(self.get_parameter("require_amcl").value)

        self.max_ages = {
            "stm": float(self.get_parameter("max_stm_age_sec").value),
            "comm": float(self.get_parameter("max_comm_age_sec").value),
            "odom": float(self.get_parameter("max_odom_age_sec").value),
            "scan": float(self.get_parameter("max_scan_age_sec").value),
            "amcl": float(self.get_parameter("max_amcl_age_sec").value),
        }

        self.status_pub = self.create_publisher(String, "/amr/safety_supervisor/status", 10)
        be = best_effort_qos()
        rel = reliable_qos()

        self.create_subscription(Int32, "/amr_stm/fault_mask", self.handle_fault_mask, be)
        self.create_subscription(UInt32, "/amr_stm/safety_state", self.handle_safety_state, be)
        self.create_subscription(String, "/amr_stm/comm_status", self.handle_comm_status, rel)
        self.create_subscription(UInt32, "/amr_stm/comm_fault_mask", self.handle_comm_fault_mask, rel)
        self.create_subscription(Odometry, self.odom_topic, self.handle_odom, rel)
        self.create_subscription(LaserScan, self.scan_topic, self.handle_scan, be)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_topic, self.handle_amcl_pose, rel)

        self.last_seen = {}
        self.fault_mask = None
        self.safety_state = None
        self.comm_status = None
        self.comm_fault_mask = None
        self.odom_speed = None
        self.last_summary_key = None

        period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(period, self.publish_status)

        if self.enforce or not self.monitor_only:
            self.get_logger().warn("Safety supervisor enforcement requested, but Step 3 is monitor-only; no stop/enable commands will be published.")
        if self.auto_reenable_when_safe:
            self.get_logger().warn("auto_reenable_when_safe is ignored in Step 3 monitor-only mode.")
        self.get_logger().info("AMR safety supervisor running in monitor-only mode.")

    def mark(self, name):
        self.last_seen[name] = time.monotonic()

    def handle_fault_mask(self, msg):
        self.mark("stm")
        self.fault_mask = int(msg.data)

    def handle_safety_state(self, msg):
        self.mark("stm")
        self.safety_state = int(msg.data)

    def handle_comm_status(self, msg):
        self.mark("comm")
        self.comm_status = msg.data

    def handle_comm_fault_mask(self, msg):
        self.mark("comm")
        self.comm_fault_mask = int(msg.data)

    def handle_odom(self, msg):
        self.mark("odom")
        vx = float(msg.twist.twist.linear.x)
        wz = float(msg.twist.twist.angular.z)
        self.odom_speed = {"linear_x": vx, "angular_z": wz}

    def handle_scan(self, _msg):
        self.mark("scan")

    def handle_amcl_pose(self, _msg):
        self.mark("amcl")

    def age(self, name, now):
        if name not in self.last_seen:
            return None
        return now - self.last_seen[name]

    def stale_flags(self, now):
        flags = {}
        for name, limit in self.max_ages.items():
            age = self.age(name, now)
            if name == "amcl" and age is None and not self.require_amcl:
                flags[name] = False
            else:
                flags[name] = age is None or age > limit
        return flags

    def decode_safety_state(self):
        if self.safety_state is None:
            return {"raw": None, "control_state": None, "fault_mask": None, "faults": []}
        control_state = (self.safety_state >> 16) & 0xFFFF
        fault_mask = self.safety_state & 0xFFFF
        return {
            "raw": self.safety_state,
            "control_state": CTRL_STATES.get(control_state, f"UNKNOWN_{control_state}"),
            "fault_mask": fault_mask,
            "faults": decode_bits(fault_mask, STM_FAULTS),
        }

    def publish_status(self):
        now = time.monotonic()
        ages = {name: self.age(name, now) for name in ["stm", "comm", "odom", "scan", "amcl"]}
        stale = self.stale_flags(now)
        stm_faults = decode_bits(self.fault_mask or 0, STM_FAULTS)
        comm_faults = decode_bits(self.comm_fault_mask or 0, COMM_FAULTS)

        healthy = (
            not any(stale.values())
            and (self.fault_mask in (None, 0))
            and (self.comm_fault_mask in (None, 0))
            and (self.comm_status in (None, "stm_link_ok"))
        )
        if self.fault_mask is None or self.comm_status is None:
            healthy = False

        status = {
            "mode": "monitor_only",
            "healthy": healthy,
            "action_authority": False,
            "stale": stale,
            "ages_sec": {name: (None if value is None else round(value, 3)) for name, value in ages.items()},
            "fault_mask": self.fault_mask,
            "faults": stm_faults,
            "safety_state": self.decode_safety_state(),
            "comm_status": self.comm_status,
            "comm_fault_mask": self.comm_fault_mask,
            "comm_faults": comm_faults,
            "odom_speed": self.odom_speed,
            "thresholds_sec": self.max_ages,
        }

        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self.status_pub.publish(msg)

        summary_key = (
            healthy,
            tuple(sorted(name for name, value in stale.items() if value)),
            self.fault_mask,
            self.comm_fault_mask,
            self.comm_status,
        )
        if summary_key != self.last_summary_key:
            self.last_summary_key = summary_key
            if healthy:
                self.get_logger().info("Safety monitor healthy.")
            else:
                self.get_logger().warn(
                    "Safety monitor unhealthy: stale=%s fault_mask=%s comm_status=%s comm_fault_mask=%s"
                    % (
                        ",".join(name for name, value in stale.items() if value) or "none",
                        self.fault_mask,
                        self.comm_status,
                        self.comm_fault_mask,
                    )
                )


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

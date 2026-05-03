#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float32, Int32, String, UInt32, UInt32MultiArray


TOPIC_LABELS = {
    "wheel": "/amr_stm/wheel_state",
    "fault": "/amr_stm/fault_mask",
    "diag": "/amr_stm/ros_diag",
    "safety": "/amr_stm/safety_state",
    "comm": "/amr_stm/comm_status",
    "comm_fault": "/amr_stm/comm_fault_mask",
    "odom": "/odom",
    "scan": "/scan",
    "amcl": "/amcl_pose",
    "cmd": "/diff_drive_controller/cmd_vel_unstamped",
    "duty_left": "/amr_stm/duty_cmd_left",
    "duty_right": "/amr_stm/duty_cmd_right",
    "current_left": "/amr_stm/current_left_ma",
    "current_right": "/amr_stm/current_right_ma",
}

RESET_BITS = [
    (1, "BOR"),
    (2, "PIN"),
    (4, "POR/PDR"),
    (8, "SOFT"),
    (16, "IWDG"),
    (32, "WWDG"),
    (64, "LPWR"),
]


def best_effort_qos():
    qos = QoSProfile(depth=10)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


def reliable_qos():
    return QoSProfile(depth=10)


class NumericRange:
    def __init__(self):
        self.min_value = None
        self.max_value = None

    def update(self, value):
        if self.min_value is None or value < self.min_value:
            self.min_value = value
        if self.max_value is None or value > self.max_value:
            self.max_value = value

    def text(self, decimals=3):
        if self.min_value is None:
            return "n/a"
        return f"{self.min_value:.{decimals}f}..{self.max_value:.{decimals}f}"


class BaselineProbe:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("amr_baseline_probe")
        self.counts = {name: 0 for name in TOPIC_LABELS}
        self.first_time = {}
        self.last_time = {}
        self.gap_limits = {
            "wheel": args.max_gap_sec,
            "fault": args.max_gap_sec,
            "odom": args.max_gap_sec,
            "scan": args.max_gap_sec,
            "diag": args.max_diag_gap_sec,
            "safety": args.max_diag_gap_sec,
            "amcl": args.max_amcl_gap_sec,
        }
        self.big_gaps = {name: [] for name in self.gap_limits}
        self.last_messages = {}
        self.boot_drops = []
        self.last_boot_ms = None
        self.cmd_linear = NumericRange()
        self.cmd_angular = NumericRange()
        self.left_duty = NumericRange()
        self.right_duty = NumericRange()
        self.left_current = NumericRange()
        self.right_current = NumericRange()
        self.wheel_velocity_abs = NumericRange()
        self.amcl_steps = []
        self.last_amcl_pose = None

        be = best_effort_qos()
        rel = reliable_qos()
        self.node.create_subscription(JointState, TOPIC_LABELS["wheel"], self.handle_wheel, be)
        self.node.create_subscription(Int32, TOPIC_LABELS["fault"], self.cb("fault"), be)
        self.node.create_subscription(UInt32MultiArray, TOPIC_LABELS["diag"], self.handle_diag, be)
        self.node.create_subscription(UInt32, TOPIC_LABELS["safety"], self.cb("safety"), be)
        self.node.create_subscription(String, TOPIC_LABELS["comm"], self.cb("comm"), rel)
        self.node.create_subscription(UInt32, TOPIC_LABELS["comm_fault"], self.cb("comm_fault"), rel)
        self.node.create_subscription(Odometry, TOPIC_LABELS["odom"], self.cb("odom"), rel)
        self.node.create_subscription(LaserScan, TOPIC_LABELS["scan"], self.cb("scan"), be)
        self.node.create_subscription(PoseWithCovarianceStamped, TOPIC_LABELS["amcl"], self.handle_amcl, rel)
        self.node.create_subscription(Twist, TOPIC_LABELS["cmd"], self.handle_cmd, rel)
        self.node.create_subscription(Float32, TOPIC_LABELS["duty_left"], self.handle_duty_left, be)
        self.node.create_subscription(Float32, TOPIC_LABELS["duty_right"], self.handle_duty_right, be)
        self.node.create_subscription(Int32, TOPIC_LABELS["current_left"], self.handle_current_left, be)
        self.node.create_subscription(Int32, TOPIC_LABELS["current_right"], self.handle_current_right, be)

    def cb(self, name):
        def callback(msg):
            self.mark(name, msg)

        return callback

    def mark(self, name, msg):
        now = time.monotonic()
        self.counts[name] += 1
        self.last_messages[name] = msg
        self.first_time.setdefault(name, now)
        if name in self.last_time and name in self.big_gaps:
            gap = now - self.last_time[name]
            if gap > self.gap_limits[name]:
                self.big_gaps[name].append(gap)
        self.last_time[name] = now

    def handle_wheel(self, msg):
        self.mark("wheel", msg)
        for value in msg.velocity:
            self.wheel_velocity_abs.update(abs(value))

    def handle_diag(self, msg):
        self.mark("diag", msg)
        data = list(msg.data)
        if len(data) >= 20:
            boot_ms = data[0]
            if self.last_boot_ms is not None and boot_ms + 1000 < self.last_boot_ms:
                self.boot_drops.append((self.last_boot_ms, boot_ms, data[18], data[19]))
            self.last_boot_ms = boot_ms

    def handle_amcl(self, msg):
        self.mark("amcl", msg)
        pose = msg.pose.pose
        current = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        if self.last_amcl_pose is not None:
            dx = current[0] - self.last_amcl_pose[0]
            dy = current[1] - self.last_amcl_pose[1]
            dyaw = wrap_angle(current[2] - self.last_amcl_pose[2])
            distance = math.hypot(dx, dy)
            if distance > self.args.max_amcl_step_m or abs(dyaw) > self.args.max_amcl_step_rad:
                self.amcl_steps.append((distance, dyaw))
        self.last_amcl_pose = current

    def handle_cmd(self, msg):
        self.mark("cmd", msg)
        self.cmd_linear.update(msg.linear.x)
        self.cmd_angular.update(msg.angular.z)

    def handle_duty_left(self, msg):
        self.mark("duty_left", msg)
        self.left_duty.update(msg.data)

    def handle_duty_right(self, msg):
        self.mark("duty_right", msg)
        self.right_duty.update(msg.data)

    def handle_current_left(self, msg):
        self.mark("current_left", msg)
        self.left_current.update(float(msg.data))

    def handle_current_right(self, msg):
        self.mark("current_right", msg)
        self.right_current.update(float(msg.data))

    def run(self):
        deadline = time.monotonic() + self.args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def rate(self, name):
        if self.counts[name] < 2:
            return 0.0
        span = self.last_time[name] - self.first_time[name]
        if span <= 0.0:
            return 0.0
        return (self.counts[name] - 1) / span

    def age(self, name):
        if name not in self.last_time:
            return None
        return time.monotonic() - self.last_time[name]

    def last_data(self, name):
        msg = self.last_messages.get(name)
        return None if msg is None else getattr(msg, "data", None)

    def print_summary(self):
        labels = [
            ("wheel", self.args.min_wheel_hz),
            ("fault", self.args.min_wheel_hz),
            ("diag", self.args.min_diag_hz),
            ("odom", self.args.min_odom_hz),
            ("scan", self.args.min_scan_hz),
            ("comm", self.args.min_comm_hz),
        ]
        print(f"duration_sec: {self.args.duration:.1f}")
        print("rates_hz:")
        for name, min_rate in labels:
            print(f"  {name}: {self.rate(name):.3f} count={self.counts[name]} min={min_rate:.3f}")
        print("big_gaps_sec:")
        for name in ["wheel", "fault", "diag", "safety", "odom", "scan", "amcl"]:
            gaps = self.big_gaps[name]
            text = [round(value, 3) for value in gaps[:20]]
            limit = self.gap_limits[name]
            print(
                f"  {name}: count={len(gaps)} limit={limit:.3f} max={max(gaps):.3f} samples={text}"
                if gaps
                else f"  {name}: count=0 limit={limit:.3f}"
            )
        print("last_values:")
        print(f"  fault_mask: {self.last_data('fault')}")
        print(f"  safety_state: {self.last_data('safety')}")
        print(f"  comm_status: {self.last_data('comm')}")
        print(f"  comm_fault_mask: {self.last_data('comm_fault')}")
        diag = self.last_messages.get("diag")
        if diag is not None:
            data = list(diag.data)
            print(f"  ros_diag: {data}")
            if len(data) >= 20:
                print(f"  boot_ms: {data[0]}")
                print(f"  reset_cause_mask: {data[18]} ({decode_reset_mask(data[18])})")
                print(f"  reset_csr_raw: 0x{data[19]:08x}")
                print(f"  uart_write_failures: {data[9]}")
                print(f"  uart_write_timeouts: {data[10]}")
                print(f"  pub_failures: {data[5]}")
        print("ranges:")
        print(f"  cmd_linear_mps: {self.cmd_linear.text()}")
        print(f"  cmd_angular_radps: {self.cmd_angular.text()}")
        print(f"  duty_left: {self.left_duty.text()}")
        print(f"  duty_right: {self.right_duty.text()}")
        print(f"  current_left_ma: {self.left_current.text(1)}")
        print(f"  current_right_ma: {self.right_current.text(1)}")
        print(f"  wheel_velocity_abs_radps: {self.wheel_velocity_abs.text()}")
        print(f"amcl_large_steps: count={len(self.amcl_steps)} samples={[(round(d, 3), round(y, 3)) for d, y in self.amcl_steps[:20]]}")
        print(f"boot_drops: count={len(self.boot_drops)} samples={[(a, b, c, hex(d)) for a, b, c, d in self.boot_drops]}")
        passed, failures = self.evaluate()
        print(f"baseline_result: {'PASS' if passed else 'FAIL'}")
        for failure in failures:
            print(f"  FAIL: {failure}")
        return 0 if passed else 1

    def evaluate(self):
        failures = []
        required_rates = {
            "wheel": self.args.min_wheel_hz,
            "fault": self.args.min_wheel_hz,
            "diag": self.args.min_diag_hz,
            "odom": self.args.min_odom_hz,
            "scan": self.args.min_scan_hz,
            "comm": self.args.min_comm_hz,
        }
        for name, minimum in required_rates.items():
            if self.rate(name) < minimum:
                failures.append(f"{TOPIC_LABELS[name]} rate {self.rate(name):.2f} Hz below {minimum:.2f} Hz")
        for name in ["wheel", "fault", "diag", "odom", "scan"]:
            if self.big_gaps[name]:
                failures.append(f"{TOPIC_LABELS[name]} had {len(self.big_gaps[name])} gap(s) > {self.gap_limits[name]:.2f}s")
        if self.last_data("fault") not in (0, None):
            failures.append(f"fault_mask is nonzero: {self.last_data('fault')}")
        if self.last_data("comm_fault") not in (0, None):
            failures.append(f"comm_fault_mask is nonzero: {self.last_data('comm_fault')}")
        if self.last_data("comm") not in ("stm_link_ok", None):
            failures.append(f"comm_status is not stm_link_ok: {self.last_data('comm')}")
        if self.boot_drops:
            failures.append("STM boot_ms dropped during observation")
        diag = self.last_messages.get("diag")
        if diag is not None:
            data = list(diag.data)
            if len(data) >= 20:
                if data[9] != 0:
                    failures.append(f"uart_write_failures is nonzero: {data[9]}")
                if data[10] != 0:
                    failures.append(f"uart_write_timeouts is nonzero: {data[10]}")
                if data[5] != 0:
                    failures.append(f"pub_failures is nonzero: {data[5]}")
        return not failures, failures

    def destroy(self):
        self.node.destroy_node()


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def decode_reset_mask(mask):
    names = [name for bit, name in RESET_BITS if mask & bit]
    return ",".join(names) if names else "none"


def parse_args():
    parser = argparse.ArgumentParser(description="Collect AMR baseline health metrics.")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--max-gap-sec", type=float, default=0.5)
    parser.add_argument("--max-diag-gap-sec", type=float, default=1.5)
    parser.add_argument("--max-amcl-gap-sec", type=float, default=2.0)
    parser.add_argument("--min-wheel-hz", type=float, default=8.0)
    parser.add_argument("--min-diag-hz", type=float, default=1.5)
    parser.add_argument("--min-odom-hz", type=float, default=40.0)
    parser.add_argument("--min-scan-hz", type=float, default=8.0)
    parser.add_argument("--min-comm-hz", type=float, default=1.5)
    parser.add_argument("--max-amcl-step-m", type=float, default=0.25)
    parser.add_argument("--max-amcl-step-rad", type=float, default=0.6)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    probe = BaselineProbe(args)
    try:
        probe.run()
        return probe.print_summary()
    finally:
        probe.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

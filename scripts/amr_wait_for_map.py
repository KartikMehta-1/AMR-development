#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapWaiter(Node):
    def __init__(self, topic):
        super().__init__("amr_wait_for_map")
        self.message = None
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, topic, self.handle_map, qos)

    def handle_map(self, message):
        self.message = message


def parse_args():
    parser = argparse.ArgumentParser(description="Wait until /map is receivable by a late subscriber.")
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--expect-width", type=int)
    parser.add_argument("--expect-height", type=int)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = MapWaiter(args.topic)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.message is None:
                continue
            info = node.message.info
            if args.expect_width is not None and info.width != args.expect_width:
                print(f"received map width {info.width}, expected {args.expect_width}", file=sys.stderr)
                return 2
            if args.expect_height is not None and info.height != args.expect_height:
                print(f"received map height {info.height}, expected {args.expect_height}", file=sys.stderr)
                return 2
            print(
                f"map ready: frame={node.message.header.frame_id} "
                f"size={info.width}x{info.height} res={info.resolution:.3f} "
                f"origin=({info.origin.position.x:.3f},{info.origin.position.y:.3f})",
                flush=True,
            )
            return 0

        print(f"timed out waiting for {args.topic}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

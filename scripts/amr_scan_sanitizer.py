#!/usr/bin/env python3

import math

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanSanitizer:
    def __init__(self):
        self.node = rclpy.create_node("amr_scan_sanitizer")
        self.input_topic = self.node.declare_parameter("input_topic", "/scan").value
        self.output_topic = self.node.declare_parameter("output_topic", "/scan_filtered").value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.node.create_publisher(LaserScan, self.output_topic, qos)
        self.sub = self.node.create_subscription(
            LaserScan,
            self.input_topic,
            self.handle_scan,
            qos,
        )

    def handle_scan(self, msg):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.intensities = msg.intensities

        filtered.ranges = [
            math.inf if math.isfinite(value) and value < msg.range_min else value
            for value in msg.ranges
        ]
        self.pub.publish(filtered)


def main():
    rclpy.init()
    sanitizer = ScanSanitizer()
    try:
        rclpy.spin(sanitizer.node)
    finally:
        sanitizer.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

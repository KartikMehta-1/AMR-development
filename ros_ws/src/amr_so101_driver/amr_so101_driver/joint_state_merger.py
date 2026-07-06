import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__("amr_joint_state_merger")
        self.declare_parameter("base_joint_states_topic", "/amr/joint_states")
        self.declare_parameter("arm_joint_states_topic", "/so101/joint_states")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("stale_timeout_sec", 1.0)

        self._latest = {}
        self._subscriptions = []
        input_topics = [
            self.get_parameter("base_joint_states_topic").value,
            self.get_parameter("arm_joint_states_topic").value,
        ]
        for topic in input_topics:
            self._subscriptions.append(
                self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, source=topic: self._store_joint_state(source, msg),
                    10,
                )
            )
            self.get_logger().info(f"Merging JointState input: {topic}")

        self._pub = self.create_publisher(
            JointState, self.get_parameter("output_topic").value, 10
        )
        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(period, self._publish_merged)

    def _store_joint_state(self, source, msg):
        self._latest[source] = (time.monotonic(), msg)

    def _publish_merged(self):
        now = time.monotonic()
        stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        positions = {}

        for _, (stamp, msg) in list(self._latest.items()):
            if now - stamp > stale_timeout:
                continue
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    positions[name] = msg.position[index]

        if not positions:
            return

        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.name = sorted(positions)
        merged.position = [positions[name] for name in merged.name]
        self._pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

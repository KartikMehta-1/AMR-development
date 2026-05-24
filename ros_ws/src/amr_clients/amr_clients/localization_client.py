from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

from amr_clients.common import TopicCache, best_effort_qos, reliable_qos, transient_local_qos


@dataclass
class LocalizationStatus:
    ready: bool
    pose_age_sec: Optional[float]
    scan_age_sec: Optional[float]
    map_to_odom_available: bool
    blockers: list[str]


class LocalizationClient:
    def __init__(
        self,
        node: Node,
        pose_topic: str = "/amcl_pose",
        scan_topic: str = "/scan",
        global_frame: str = "map",
        odom_frame: str = "odom",
    ):
        self.node = node
        self.pose_topic = pose_topic
        self.scan_topic = scan_topic
        self.global_frame = global_frame
        self.odom_frame = odom_frame
        self.cache = TopicCache(node)
        self.cache.subscribe(pose_topic, PoseWithCovarianceStamped, transient_local_qos())
        self.cache.subscribe(scan_topic, LaserScan, best_effort_qos())
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

    def map_to_odom_available(self, timeout_sec: float = 0.1) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                self.global_frame,
                self.odom_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )
            return True
        except Exception:
            return False

    def wait_for_status(self, timeout_sec: float = 2.0) -> bool:
        return self.cache.wait_for([self.pose_topic, self.scan_topic], timeout_sec)

    def status(self, max_pose_age_sec: float = 3.0, max_scan_age_sec: float = 1.0) -> LocalizationStatus:
        pose = self.cache.get(self.pose_topic)
        scan = self.cache.get(self.scan_topic)
        pose_age = None if pose is None else pose.age_sec()
        scan_age = None if scan is None else scan.age_sec()
        has_tf = self.map_to_odom_available()
        blockers: list[str] = []
        if pose_age is None:
            blockers.append("amcl_pose_missing")
        elif pose_age > max_pose_age_sec:
            blockers.append("amcl_pose_stale")
        if scan_age is None:
            blockers.append("scan_missing")
        elif scan_age > max_scan_age_sec:
            blockers.append("scan_stale")
        if not has_tf:
            blockers.append("map_to_odom_missing")
        return LocalizationStatus(
            ready=not blockers,
            pose_age_sec=pose_age,
            scan_age_sec=scan_age,
            map_to_odom_available=has_tf,
            blockers=blockers,
        )


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class ClientResult:
    ok: bool
    message: str = ""
    data: Any = None
    blockers: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass
class TopicSample:
    topic: str
    data: Any
    stamp_monotonic: float

    def age_sec(self) -> float:
        return time.monotonic() - self.stamp_monotonic


def best_effort_qos(depth: int = 10) -> QoSProfile:
    qos = QoSProfile(depth=depth)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(depth=depth)


def transient_local_qos(depth: int = 10) -> QoSProfile:
    qos = QoSProfile(depth=depth)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


def wait_for_response(node: Node, future, service_name: str, timeout_sec: float) -> ClientResult:
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        return ClientResult(False, f"timed out waiting for {service_name}")
    response = future.result()
    if response is None:
        return ClientResult(False, f"no response from {service_name}")
    return ClientResult(True, data=response)


def call_service(node: Node, client, request, service_name: str, timeout_sec: float) -> ClientResult:
    if not client.wait_for_service(timeout_sec=timeout_sec):
        return ClientResult(False, f"service unavailable: {service_name}")
    return wait_for_response(node, client.call_async(request), service_name, timeout_sec)


def spin_until(
    node: Node,
    predicate,
    timeout_sec: float,
    spin_timeout_sec: float = 0.05,
) -> bool:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=spin_timeout_sec)
    return bool(predicate())


def decode_bits(mask: int, table: list[tuple[int, str]]) -> list[str]:
    names = [name for bit, name in table if mask & bit]
    known = 0
    for bit, _name in table:
        known |= bit
    unknown = mask & ~known
    if unknown:
        names.append(f"UNKNOWN_BITS_0x{unknown:x}")
    return names or ["none"]


class TopicCache:
    def __init__(self, node: Node):
        self.node = node
        self.samples: dict[str, TopicSample] = {}
        self.subscriptions = []

    def subscribe(self, topic: str, msg_type, qos: Optional[QoSProfile] = None) -> None:
        self.subscriptions.append(
            self.node.create_subscription(msg_type, topic, self._callback(topic), qos or reliable_qos())
        )

    def _callback(self, topic: str):
        def callback(msg) -> None:
            self.samples[topic] = TopicSample(topic=topic, data=msg, stamp_monotonic=time.monotonic())

        return callback

    def get(self, topic: str) -> Optional[TopicSample]:
        return self.samples.get(topic)

    def wait_for(self, topics: list[str], timeout_sec: float) -> bool:
        return spin_until(self.node, lambda: all(topic in self.samples for topic in topics), timeout_sec)

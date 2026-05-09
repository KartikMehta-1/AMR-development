from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32, String, UInt32, UInt32MultiArray

from amr_clients.common import TopicCache, best_effort_qos, decode_bits, reliable_qos


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


@dataclass
class StmDiagnostics:
    fault_mask: Optional[int] = None
    fault_names: list[str] = field(default_factory=list)
    safety_state: Optional[int] = None
    comm_ok: Optional[bool] = None
    comm_status: Optional[str] = None
    comm_fault_mask: Optional[int] = None
    comm_fault_names: list[str] = field(default_factory=list)
    wheel_state_age_sec: Optional[float] = None
    current_left_ma: Optional[int] = None
    current_right_ma: Optional[int] = None
    ros_diag: Optional[list[int]] = None

    @property
    def healthy(self) -> bool:
        return (
            self.fault_mask == 0
            and self.comm_fault_mask == 0
            and self.comm_status == "stm_link_ok"
            and self.wheel_state_age_sec is not None
        )


class StmDiagnosticsClient:
    """Read-only topic client for `/amr_stm/*` diagnostics."""

    def __init__(self, node: Node):
        self.node = node
        self.cache = TopicCache(node)
        be = best_effort_qos()
        rel = reliable_qos()
        self.cache.subscribe("/amr_stm/wheel_state", JointState, be)
        self.cache.subscribe("/amr_stm/fault_mask", Int32, be)
        self.cache.subscribe("/amr_stm/safety_state", UInt32, be)
        self.cache.subscribe("/amr_stm/current_left_ma", Int32, be)
        self.cache.subscribe("/amr_stm/current_right_ma", Int32, be)
        self.cache.subscribe("/amr_stm/ros_diag", UInt32MultiArray, be)
        self.cache.subscribe("/amr_stm/comm_ok", Bool, rel)
        self.cache.subscribe("/amr_stm/comm_status", String, rel)
        self.cache.subscribe("/amr_stm/comm_fault_mask", UInt32, rel)

    def wait_for_core_topics(self, timeout_sec: float = 2.0) -> bool:
        return self.cache.wait_for(
            [
                "/amr_stm/wheel_state",
                "/amr_stm/fault_mask",
                "/amr_stm/comm_status",
                "/amr_stm/comm_fault_mask",
            ],
            timeout_sec,
        )

    def snapshot(self) -> StmDiagnostics:
        fault = self.cache.get("/amr_stm/fault_mask")
        safety = self.cache.get("/amr_stm/safety_state")
        comm_ok = self.cache.get("/amr_stm/comm_ok")
        comm_status = self.cache.get("/amr_stm/comm_status")
        comm_fault = self.cache.get("/amr_stm/comm_fault_mask")
        wheel = self.cache.get("/amr_stm/wheel_state")
        current_left = self.cache.get("/amr_stm/current_left_ma")
        current_right = self.cache.get("/amr_stm/current_right_ma")
        ros_diag = self.cache.get("/amr_stm/ros_diag")

        fault_mask = None if fault is None else int(fault.data.data)
        comm_fault_mask = None if comm_fault is None else int(comm_fault.data.data)
        return StmDiagnostics(
            fault_mask=fault_mask,
            fault_names=[] if fault_mask is None else decode_bits(fault_mask, STM_FAULTS),
            safety_state=None if safety is None else int(safety.data.data),
            comm_ok=None if comm_ok is None else bool(comm_ok.data.data),
            comm_status=None if comm_status is None else str(comm_status.data.data),
            comm_fault_mask=comm_fault_mask,
            comm_fault_names=[] if comm_fault_mask is None else decode_bits(comm_fault_mask, COMM_FAULTS),
            wheel_state_age_sec=None if wheel is None else wheel.age_sec(),
            current_left_ma=None if current_left is None else int(current_left.data.data),
            current_right_ma=None if current_right is None else int(current_right.data.data),
            ros_diag=None if ros_diag is None else list(ros_diag.data.data),
        )

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Optional

from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from amr_clients.common import ClientResult, TopicCache, call_service, reliable_qos
from amr_clients.stm_diagnostics_client import StmDiagnostics, StmDiagnosticsClient


@dataclass
class SafetySnapshot:
    stm: StmDiagnostics
    supervisor_status: Optional[dict[str, Any]] = None
    blockers: list[str] = field(default_factory=list)

    @property
    def healthy(self) -> bool:
        supervisor_healthy = None
        if self.supervisor_status is not None:
            supervisor_healthy = bool(self.supervisor_status.get("healthy", False))
        return self.stm.healthy and supervisor_healthy is True and not self.blockers


class SafetyClient:
    """Shared read-mostly client for STM and safety-supervisor state."""

    def __init__(self, node: Node):
        self.node = node
        self.stm = StmDiagnosticsClient(node)
        self.cache = TopicCache(node)
        self.cache.subscribe("/amr/safety_supervisor/status", String, reliable_qos())
        self.reset_client = node.create_client(Trigger, "/amr/safety_supervisor/reset_intervention")

    def wait_for_status(self, timeout_sec: float = 2.0) -> bool:
        self.stm.wait_for_core_topics(timeout_sec)
        return self.cache.wait_for(["/amr/safety_supervisor/status"], timeout_sec)

    def snapshot(self) -> SafetySnapshot:
        stm = self.stm.snapshot()
        supervisor_sample = self.cache.get("/amr/safety_supervisor/status")
        supervisor_status = None
        blockers: list[str] = []
        if supervisor_sample is not None:
            try:
                supervisor_status = json.loads(supervisor_sample.data.data)
            except json.JSONDecodeError:
                blockers.append("safety_supervisor_status_invalid_json")
        else:
            blockers.append("safety_supervisor_status_missing")

        if stm.fault_mask not in (0, None):
            blockers.append("stm_fault_mask_nonzero")
        if stm.comm_fault_mask not in (0, None):
            blockers.append("comm_fault_mask_nonzero")
        if stm.comm_status not in (None, "stm_link_ok"):
            blockers.append(f"comm_status_{stm.comm_status}")
        if supervisor_status is not None and supervisor_status.get("intervention_active"):
            blockers.append("safety_intervention_active")

        return SafetySnapshot(stm=stm, supervisor_status=supervisor_status, blockers=blockers)

    def can_reset_intervention(self) -> ClientResult:
        snapshot = self.snapshot()
        if snapshot.stm.fault_mask not in (0, None):
            return ClientResult(
                False,
                "cannot reset while STM fault mask is nonzero",
                data=snapshot,
                blockers=["stm_fault_mask_nonzero"],
            )
        if snapshot.stm.comm_fault_mask not in (0, None):
            return ClientResult(
                False,
                "cannot reset while STM communication fault mask is nonzero",
                data=snapshot,
                blockers=["comm_fault_mask_nonzero"],
            )
        if snapshot.supervisor_status is None:
            return ClientResult(
                False,
                "cannot reset without safety supervisor status",
                data=snapshot,
                blockers=["safety_supervisor_status_missing"],
            )
        if not bool(snapshot.supervisor_status.get("healthy", False)):
            return ClientResult(
                False,
                "cannot reset while safety supervisor reports unsafe state",
                data=snapshot,
                blockers=snapshot.blockers or ["safety_supervisor_unhealthy"],
            )
        return ClientResult(True, "safety intervention reset is allowed", data=snapshot)

    def request_supervisor_reset(self, timeout_sec: float = 5.0, require_safe: bool = True) -> ClientResult:
        if require_safe:
            allowed = self.can_reset_intervention()
            if not allowed.ok:
                return allowed
        result = call_service(
            self.node,
            self.reset_client,
            Trigger.Request(),
            "/amr/safety_supervisor/reset_intervention",
            timeout_sec,
        )
        if not result.ok:
            return result
        return ClientResult(bool(result.data.success), result.data.message, data=result.data)

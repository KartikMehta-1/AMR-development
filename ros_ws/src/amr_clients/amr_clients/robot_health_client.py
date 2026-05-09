from __future__ import annotations

from dataclasses import dataclass, field

from rclpy.node import Node

from amr_clients.localization_client import LocalizationClient, LocalizationStatus
from amr_clients.safety_client import SafetyClient, SafetySnapshot


@dataclass
class RobotHealth:
    ok: bool
    blockers: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    safety: SafetySnapshot | None = None
    localization: LocalizationStatus | None = None


class RobotHealthClient:
    def __init__(self, node: Node):
        self.node = node
        self.safety = SafetyClient(node)
        self.localization = LocalizationClient(node)

    def snapshot(
        self,
        require_localization: bool = True,
        max_pose_age_sec: float = 3.0,
        max_scan_age_sec: float = 1.0,
    ) -> RobotHealth:
        safety = self.safety.snapshot()
        localization = self.localization.status(
            max_pose_age_sec=max_pose_age_sec,
            max_scan_age_sec=max_scan_age_sec,
        )
        blockers: list[str] = []
        warnings: list[str] = []

        if safety.stm.fault_mask is None:
            warnings.append("stm_fault_mask_unknown")
        elif safety.stm.fault_mask != 0:
            blockers.append("stm_fault_mask_nonzero")
        if safety.stm.comm_fault_mask is None:
            warnings.append("comm_fault_mask_unknown")
        elif safety.stm.comm_fault_mask != 0:
            blockers.append("comm_fault_mask_nonzero")
        if safety.supervisor_status is None:
            warnings.append("safety_supervisor_status_unknown")
        elif not safety.supervisor_status.get("healthy", False):
            blockers.append("safety_supervisor_unhealthy")

        if require_localization and not localization.ready:
            blockers.extend(localization.blockers)

        return RobotHealth(
            ok=not blockers,
            blockers=sorted(set(blockers)),
            warnings=sorted(set(warnings)),
            safety=safety,
            localization=localization,
        )

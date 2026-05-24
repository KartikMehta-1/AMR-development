from __future__ import annotations

from dataclasses import dataclass, field

from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import GetState
from rclpy.node import Node

from amr_clients.common import ClientResult, call_service


@dataclass
class Nav2Status:
    lifecycle_states: dict[str, str] = field(default_factory=dict)
    blockers: list[str] = field(default_factory=list)

    @property
    def active(self) -> bool:
        return bool(self.lifecycle_states) and not self.blockers


class NavigationClient:
    """Read-only and cancel-only client for Nav2 diagnostics."""

    def __init__(self, node: Node):
        self.node = node
        self.cancel_client = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.lifecycle_clients = {}

    def get_lifecycle_state(self, node_name: str, timeout_sec: float = 2.0) -> ClientResult:
        client = self.lifecycle_clients.get(node_name)
        if client is None:
            client = self.node.create_client(GetState, f"{node_name}/get_state")
            self.lifecycle_clients[node_name] = client
        result = call_service(self.node, client, GetState.Request(), f"{node_name}/get_state", timeout_sec)
        if not result.ok:
            return result
        state = result.data.current_state
        return ClientResult(True, state.label, data=state)

    def get_nav2_status(
        self,
        nodes: list[str] | None = None,
        timeout_sec: float = 2.0,
    ) -> Nav2Status:
        nodes = nodes or [
            "/map_server",
            "/amcl",
            "/bt_navigator",
            "/planner_server",
            "/controller_server",
            "/recoveries_server",
        ]
        status = Nav2Status()
        for node_name in nodes:
            result = self.get_lifecycle_state(node_name, timeout_sec=timeout_sec)
            if result.ok:
                status.lifecycle_states[node_name] = result.message
                if result.message != "active":
                    status.blockers.append(f"{node_name}_not_active")
            else:
                status.lifecycle_states[node_name] = "unknown"
                status.blockers.append(f"{node_name}_state_unavailable")
        return status

    def cancel_nav_goal(self, timeout_sec: float = 5.0) -> ClientResult:
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        result = call_service(
            self.node,
            self.cancel_client,
            request,
            "/navigate_to_pose/_action/cancel_goal",
            timeout_sec,
        )
        if not result.ok:
            return result
        return ClientResult(True, "Nav2 cancel requested", data=result.data)

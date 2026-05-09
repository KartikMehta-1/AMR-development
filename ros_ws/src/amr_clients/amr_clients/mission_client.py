from __future__ import annotations

from typing import Iterable, Optional

from rclpy.node import Node
from std_srvs.srv import Trigger

from amr_clients.common import ClientResult, call_service
from amr_missions_msgs.srv import GetMissionState, GoToNamedPose, ListPlaces, PatrolNamedPoses


class MissionClient:
    """Thin shared client for the typed AMR mission runtime."""

    def __init__(self, node: Node, namespace: str = "/amr_missions"):
        self.node = node
        self.namespace = namespace.rstrip("/")
        self.list_places_client = node.create_client(ListPlaces, f"{self.namespace}/list_places")
        self.go_to_client = node.create_client(GoToNamedPose, f"{self.namespace}/go_to")
        self.patrol_client = node.create_client(PatrolNamedPoses, f"{self.namespace}/patrol")
        self.state_client = node.create_client(GetMissionState, f"{self.namespace}/state")
        self.cancel_client = node.create_client(Trigger, f"{self.namespace}/cancel")

    def list_places(self, timeout_sec: float = 2.0) -> ClientResult:
        result = call_service(
            self.node,
            self.list_places_client,
            ListPlaces.Request(),
            f"{self.namespace}/list_places",
            timeout_sec,
        )
        if not result.ok:
            return result
        return ClientResult(True, "places received", data=list(result.data.places))

    def get_mission_state(self, timeout_sec: float = 2.0) -> ClientResult:
        result = call_service(
            self.node,
            self.state_client,
            GetMissionState.Request(),
            f"{self.namespace}/state",
            timeout_sec,
        )
        if not result.ok:
            return result
        return ClientResult(True, "mission state received", data=result.data.status)

    def go_to_named_place(
        self,
        place: str,
        goal_timeout_sec: float = 180.0,
        service_timeout_sec: float = 10.0,
    ) -> ClientResult:
        if not place:
            return ClientResult(False, "place name is required", blockers=["empty_place"])
        request = GoToNamedPose.Request()
        request.place = place
        request.timeout_sec = float(goal_timeout_sec)
        result = call_service(
            self.node,
            self.go_to_client,
            request,
            f"{self.namespace}/go_to",
            service_timeout_sec,
        )
        if not result.ok:
            return result
        return ClientResult(bool(result.data.success), result.data.message, data=result.data)

    def patrol_route(
        self,
        places: Iterable[str],
        loops: int = 1,
        goal_timeout_sec: float = 180.0,
        retries: int = 1,
        return_home: Optional[str] = None,
        service_timeout_sec: float = 10.0,
    ) -> ClientResult:
        place_list = list(places)
        if not place_list:
            return ClientResult(False, "at least one patrol place is required", blockers=["empty_patrol"])
        request = PatrolNamedPoses.Request()
        request.places = place_list
        request.loops = int(loops)
        request.timeout_sec = float(goal_timeout_sec)
        request.retries = int(retries)
        request.return_home = return_home or ""
        result = call_service(
            self.node,
            self.patrol_client,
            request,
            f"{self.namespace}/patrol",
            service_timeout_sec,
        )
        if not result.ok:
            return result
        return ClientResult(bool(result.data.success), result.data.message, data=result.data)

    def cancel_mission(self, timeout_sec: float = 10.0) -> ClientResult:
        result = call_service(
            self.node,
            self.cancel_client,
            Trigger.Request(),
            f"{self.namespace}/cancel",
            timeout_sec,
        )
        if not result.ok:
            return result
        return ClientResult(bool(result.data.success), result.data.message, data=result.data)

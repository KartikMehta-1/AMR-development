#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import asdict, is_dataclass
from typing import Any, Callable

import rclpy

from amr_clients.localization_client import LocalizationClient
from amr_clients.mission_client import MissionClient
from amr_clients.navigation_client import NavigationClient
from amr_clients.safety_client import SafetyClient


SERVER_NAME = "amr-mission-control"
SERVER_VERSION = "0.1.0"
DEFAULT_TIMEOUT_SEC = float(os.environ.get("AMR_MISSION_MCP_TIMEOUT_SEC", "5.0"))
DEFAULT_GOAL_TIMEOUT_SEC = float(os.environ.get("AMR_MISSION_MCP_GOAL_TIMEOUT_SEC", "180.0"))
DEFAULT_MAX_POSE_AGE_SEC = float(os.environ.get("AMR_MISSION_MCP_MAX_POSE_AGE_SEC", "3.0"))
DEFAULT_MAX_SCAN_AGE_SEC = float(os.environ.get("AMR_MISSION_MCP_MAX_SCAN_AGE_SEC", "1.0"))


def to_jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return {key: to_jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [to_jsonable(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, "get_fields_and_field_types"):
        return {
            field: to_jsonable(getattr(value, field))
            for field in value.get_fields_and_field_types().keys()
        }
    return str(value)


def response(
    ok_value: bool,
    message: str,
    data: Any = None,
    blockers: list[str] | None = None,
    warnings: list[str] | None = None,
) -> dict[str, Any]:
    return {
        "available": True,
        "ok": bool(ok_value),
        "message": message,
        "data": to_jsonable(data),
        "blockers": list(blockers or []),
        "warnings": list(warnings or []),
    }


def client_result_payload(result) -> dict[str, Any]:
    return response(
        bool(result.ok),
        result.message,
        data=result.data,
        blockers=list(result.blockers),
        warnings=list(result.warnings),
    )


class RosMissionControl:
    def __init__(self) -> None:
        self.started = False
        self.node = None
        self.mission = None
        self.safety = None
        self.localization = None
        self.navigation = None

    def start(self) -> None:
        if self.started:
            return
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node(f"amr_mission_control_mcp_{os.getpid()}")
        self.mission = MissionClient(self.node)
        self.safety = SafetyClient(self.node)
        self.localization = LocalizationClient(self.node)
        self.navigation = NavigationClient(self.node)
        self.started = True
        self.spin_for(float(os.environ.get("AMR_MISSION_MCP_GRAPH_WARMUP_SEC", "1.0")))

    def spin_for(self, timeout_sec: float) -> None:
        self.start()
        remaining = max(0.0, timeout_sec)
        while remaining > 0.0:
            step = min(0.05, remaining)
            rclpy.spin_once(self.node, timeout_sec=step)
            remaining -= step

    def shutdown(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        self.node = None
        self.started = False
        if rclpy.ok():
            rclpy.shutdown()


class MissionControlTools:
    IDLE_STATES = {"idle", "succeeded", "cancelled"}

    def __init__(self) -> None:
        self.ros = RosMissionControl()

    def list_named_places(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_for(0.1)
        result = self.ros.mission.list_places(timeout_sec=float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC)))
        return client_result_payload(result)

    def get_mission_state(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_for(0.1)
        result = self.ros.mission.get_mission_state(
            timeout_sec=float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC))
        )
        return client_result_payload(result)

    def check_go_to_readiness(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_for(float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC)))
        return self._go_to_readiness(arguments)

    def go_to_named_place(self, arguments: dict[str, Any]) -> dict[str, Any]:
        place = str(arguments.get("place", "")).strip()
        timeout_sec = float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC))
        goal_timeout_sec = float(arguments.get("goal_timeout_sec", DEFAULT_GOAL_TIMEOUT_SEC))
        dry_run = bool(arguments.get("dry_run", False))
        operator_confirmed = bool(arguments.get("operator_confirmed_supervised", False))

        self.ros.spin_for(timeout_sec)
        readiness = self._go_to_readiness(arguments)
        if not readiness["ok"]:
            return readiness
        if dry_run:
            return response(True, f"dry run passed for go_to '{place}'", data=readiness["data"])
        if not operator_confirmed:
            return response(
                False,
                "operator_confirmed_supervised must be true before starting a live mission",
                data=readiness["data"],
                blockers=["operator_confirmation_required"],
            )

        result = self.ros.mission.go_to_named_place(
            place=place,
            goal_timeout_sec=goal_timeout_sec,
            service_timeout_sec=timeout_sec,
        )
        return client_result_payload(result)

    def cancel_mission(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_for(0.1)
        result = self.ros.mission.cancel_mission(timeout_sec=float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC)))
        return client_result_payload(result)

    def _go_to_readiness(self, arguments: dict[str, Any]) -> dict[str, Any]:
        place = str(arguments.get("place", "")).strip()
        timeout_sec = float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC))
        max_pose_age_sec = float(arguments.get("max_pose_age_sec", DEFAULT_MAX_POSE_AGE_SEC))
        max_scan_age_sec = float(arguments.get("max_scan_age_sec", DEFAULT_MAX_SCAN_AGE_SEC))
        blockers: list[str] = []
        warnings: list[str] = []
        data: dict[str, Any] = {"place": place}

        if not place:
            blockers.append("empty_place")

        places_result = self.ros.mission.list_places(timeout_sec=timeout_sec)
        data["places"] = to_jsonable(places_result.data)
        if not places_result.ok:
            blockers.append("mission_places_unavailable")
            warnings.extend(places_result.warnings)
        elif place and place not in places_result.data:
            blockers.append("unknown_place")

        state_result = self.ros.mission.get_mission_state(timeout_sec=timeout_sec)
        data["mission_state"] = to_jsonable(state_result.data)
        if not state_result.ok:
            blockers.append("mission_state_unavailable")
            warnings.extend(state_result.warnings)
        else:
            state = getattr(state_result.data, "state", "")
            if state not in self.IDLE_STATES:
                blockers.append("mission_not_idle")

        self.ros.safety.wait_for_status(timeout_sec=timeout_sec)
        safety = self.ros.safety.snapshot()
        data["safety"] = to_jsonable(safety)
        if not safety.healthy:
            blockers.extend(safety.blockers or ["safety_not_healthy"])

        self.ros.localization.wait_for_status(timeout_sec=timeout_sec)
        localization = self.ros.localization.status(
            max_pose_age_sec=max_pose_age_sec,
            max_scan_age_sec=max_scan_age_sec,
        )
        data["localization"] = to_jsonable(localization)
        if not localization.ready:
            blockers.extend(localization.blockers)

        nav = self.ros.navigation.get_nav2_status(timeout_sec=timeout_sec)
        data["navigation"] = to_jsonable(nav)
        if not nav.active:
            blockers.extend(nav.blockers or ["navigation_not_active"])

        blockers = sorted(set(blockers))
        warnings = sorted(set(warnings))
        if blockers:
            return response(False, "go_to readiness blocked", data=data, blockers=blockers, warnings=warnings)
        return response(True, f"go_to readiness passed for '{place}'", data=data, warnings=warnings)


TOOL_DEFINITIONS = [
    {
        "name": "list_named_places",
        "description": "Read mission named places. Does not command motion.",
        "inputSchema": {"type": "object", "properties": {"timeout_sec": {"type": "number"}}},
    },
    {
        "name": "get_mission_state",
        "description": "Read current mission status. Does not command motion.",
        "inputSchema": {"type": "object", "properties": {"timeout_sec": {"type": "number"}}},
    },
    {
        "name": "check_go_to_readiness",
        "description": "Check whether a named-place go_to mission is allowed. Does not command motion.",
        "inputSchema": {
            "type": "object",
            "required": ["place"],
            "properties": {
                "place": {"type": "string"},
                "timeout_sec": {"type": "number", "default": DEFAULT_TIMEOUT_SEC},
                "max_pose_age_sec": {"type": "number", "default": DEFAULT_MAX_POSE_AGE_SEC},
                "max_scan_age_sec": {"type": "number", "default": DEFAULT_MAX_SCAN_AGE_SEC},
            },
        },
    },
    {
        "name": "go_to_named_place",
        "description": "Start a mission_server go_to for a named place after readiness checks. Does not publish raw motion.",
        "inputSchema": {
            "type": "object",
            "required": ["place", "operator_confirmed_supervised"],
            "properties": {
                "place": {"type": "string"},
                "operator_confirmed_supervised": {
                    "type": "boolean",
                    "description": "Must be true only after the human operator confirms supervised mission execution.",
                    "default": False,
                },
                "dry_run": {"type": "boolean", "default": False},
                "timeout_sec": {"type": "number", "default": DEFAULT_TIMEOUT_SEC},
                "goal_timeout_sec": {"type": "number", "default": DEFAULT_GOAL_TIMEOUT_SEC},
                "max_pose_age_sec": {"type": "number", "default": DEFAULT_MAX_POSE_AGE_SEC},
                "max_scan_age_sec": {"type": "number", "default": DEFAULT_MAX_SCAN_AGE_SEC},
            },
        },
    },
    {
        "name": "cancel_mission",
        "description": "Request mission_server cancellation of the active mission. Does not clear faults or re-enable STM.",
        "inputSchema": {"type": "object", "properties": {"timeout_sec": {"type": "number"}}},
    },
]


class McpServer:
    def __init__(self) -> None:
        self.tools = MissionControlTools()
        self.tool_handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
            "list_named_places": self.tools.list_named_places,
            "get_mission_state": self.tools.get_mission_state,
            "check_go_to_readiness": self.tools.check_go_to_readiness,
            "go_to_named_place": self.tools.go_to_named_place,
            "cancel_mission": self.tools.cancel_mission,
        }

    def handle(self, request: dict[str, Any]) -> dict[str, Any] | None:
        method = request.get("method")
        request_id = request.get("id")
        if method == "notifications/initialized":
            return None
        try:
            if method == "initialize":
                result = {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
                }
            elif method == "tools/list":
                result = {"tools": TOOL_DEFINITIONS}
            elif method == "tools/call":
                params = request.get("params") or {}
                name = params.get("name")
                arguments = params.get("arguments") or {}
                if name not in self.tool_handlers:
                    raise ValueError(f"unknown tool: {name}")
                payload = self.tool_handlers[name](arguments)
                result = {
                    "content": [{"type": "text", "text": json.dumps(payload, sort_keys=True)}],
                    "isError": not bool(payload.get("ok", False)),
                }
            else:
                return self.error(request_id, -32601, f"method not found: {method}")
            return {"jsonrpc": "2.0", "id": request_id, "result": result}
        except Exception as exc:
            return self.error(request_id, -32000, str(exc))

    @staticmethod
    def error(request_id: Any, code: int, message: str) -> dict[str, Any]:
        return {"jsonrpc": "2.0", "id": request_id, "error": {"code": code, "message": message}}

    def serve(self) -> None:
        try:
            while True:
                request = self.read_message()
                if request is None:
                    break
                response_message = self.handle(request)
                if response_message is not None:
                    self.write_message(response_message)
        finally:
            self.tools.ros.shutdown()

    @staticmethod
    def read_message() -> dict[str, Any] | None:
        headers: dict[str, str] = {}
        while True:
            line = sys.stdin.buffer.readline()
            if line == b"":
                return None
            line = line.decode("utf-8").strip()
            if not line:
                break
            key, value = line.split(":", 1)
            headers[key.lower()] = value.strip()
        length = int(headers.get("content-length", "0"))
        if length <= 0:
            return None
        return json.loads(sys.stdin.buffer.read(length).decode("utf-8"))

    @staticmethod
    def write_message(message: dict[str, Any]) -> None:
        body = json.dumps(message, separators=(",", ":")).encode("utf-8")
        sys.stdout.buffer.write(f"Content-Length: {len(body)}\r\n\r\n".encode("utf-8"))
        sys.stdout.buffer.write(body)
        sys.stdout.buffer.flush()


def main() -> None:
    McpServer().serve()


if __name__ == "__main__":
    main()

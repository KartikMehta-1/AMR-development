#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Callable

import rclpy

from amr_clients.localization_client import LocalizationClient
from amr_clients.mission_client import MissionClient
from amr_clients.navigation_client import NavigationClient
from amr_clients.robot_health_client import RobotHealthClient
from amr_clients.safety_client import SafetyClient
from amr_clients.stm_diagnostics_client import StmDiagnosticsClient


SERVER_NAME = "amr-state-inspection"
SERVER_VERSION = "0.1.0"
DEFAULT_TIMEOUT_SEC = float(os.environ.get("AMR_MCP_TIMEOUT_SEC", "3.0"))


def default_last_place_path() -> Path:
    configured = os.environ.get("AMR_MISSION_LAST_PLACE_PATH")
    if configured:
        return Path(configured)
    workspace_path = Path("/workspaces/AMR-development/ros_ws/log/amr_last_place.json")
    if workspace_path.parent.exists() or Path("/workspaces/AMR-development").exists():
        return workspace_path
    return Path.home() / ".ros" / "amr_last_place.json"


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


def ok(data: Any, message: str) -> dict[str, Any]:
    return {
        "available": True,
        "ok": True,
        "message": message,
        "data": to_jsonable(data),
        "blockers": [],
        "warnings": [],
    }


def observed(data: Any, ok_value: bool, message: str, blockers: list[str], warnings: list[str] | None = None) -> dict[str, Any]:
    return {
        "available": True,
        "ok": bool(ok_value),
        "message": message,
        "data": to_jsonable(data),
        "blockers": list(blockers),
        "warnings": list(warnings or []),
    }


def client_result_payload(result) -> dict[str, Any]:
    return {
        "available": bool(result.ok),
        "ok": bool(result.ok),
        "message": result.message,
        "data": to_jsonable(result.data),
        "blockers": list(result.blockers),
        "warnings": list(result.warnings),
    }


class RosState:
    def __init__(self) -> None:
        self.started = False
        self.node = None
        self.mission = None
        self.safety = None
        self.localization = None
        self.navigation = None
        self.robot_health = None
        self.stm = None

    def start(self) -> None:
        if self.started:
            return
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node(f"amr_state_inspection_mcp_{os.getpid()}")
        self.mission = MissionClient(self.node)
        self.safety = SafetyClient(self.node)
        self.localization = LocalizationClient(self.node)
        self.navigation = NavigationClient(self.node)
        self.robot_health = RobotHealthClient(self.node)
        self.stm = StmDiagnosticsClient(self.node)
        self.started = True
        deadline = time.monotonic() + float(os.environ.get("AMR_MCP_GRAPH_WARMUP_SEC", "1.0"))
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def spin_for(self, timeout_sec: float) -> None:
        self.start()
        remaining = max(0.0, timeout_sec)
        while remaining > 0.0:
            step = min(0.05, remaining)
            rclpy.spin_once(self.node, timeout_sec=step)
            remaining -= step

    def spin_once(self) -> None:
        self.start()
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def shutdown(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        self.node = None
        self.started = False
        if rclpy.ok():
            rclpy.shutdown()


class AmrStateTools:
    def __init__(self) -> None:
        self.ros = RosState()

    def get_robot_health(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_for(float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC)))
        health = self.ros.robot_health.snapshot(
            require_localization=bool(arguments.get("require_localization", True)),
            max_pose_age_sec=float(arguments.get("max_pose_age_sec", 3.0)),
            max_scan_age_sec=float(arguments.get("max_scan_age_sec", 1.0)),
        )
        return observed(health, health.ok, "robot health snapshot", health.blockers, health.warnings)

    def get_safety_state(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_for(float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC)))
        snapshot = self.ros.safety.snapshot()
        return observed(snapshot, snapshot.healthy, "safety snapshot", snapshot.blockers)

    def get_localization_state(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_for(float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC)))
        status = self.ros.localization.status(
            max_pose_age_sec=float(arguments.get("max_pose_age_sec", 3.0)),
            max_scan_age_sec=float(arguments.get("max_scan_age_sec", 1.0)),
        )
        return observed(status, status.ready, "localization snapshot", status.blockers)

    def get_mission_state(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_once()
        result = self.ros.mission.get_mission_state(
            timeout_sec=float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC))
        )
        return client_result_payload(result)

    def list_named_places(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_once()
        result = self.ros.mission.list_places(
            timeout_sec=float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC))
        )
        return client_result_payload(result)

    def get_stm_diagnostics(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_for(float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC)))
        diagnostics = self.ros.stm.snapshot()
        blockers = []
        if diagnostics.fault_mask is None:
            blockers.append("stm_fault_mask_missing")
        if diagnostics.comm_fault_mask is None:
            blockers.append("comm_fault_mask_missing")
        if diagnostics.comm_status is None:
            blockers.append("comm_status_missing")
        if diagnostics.wheel_state_age_sec is None:
            blockers.append("wheel_state_missing")
        if diagnostics.fault_mask not in (0, None):
            blockers.append("stm_fault_mask_nonzero")
        if diagnostics.comm_fault_mask not in (0, None):
            blockers.append("comm_fault_mask_nonzero")
        return observed(diagnostics, diagnostics.healthy, "STM diagnostics snapshot", blockers)

    def get_navigation_state(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self.ros.spin_once()
        nodes = arguments.get("nodes")
        if nodes is not None and not isinstance(nodes, list):
            return {
                "available": False,
                "ok": False,
                "message": "nodes must be a list",
                "data": None,
                "blockers": ["invalid_nodes_argument"],
                "warnings": [],
            }
        status = self.ros.navigation.get_nav2_status(
            nodes=nodes,
            timeout_sec=float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC)),
        )
        return observed(status, status.active, "navigation lifecycle snapshot", status.blockers)

    def get_last_known_place(self, arguments: dict[str, Any]) -> dict[str, Any]:
        path = Path(str(arguments.get("path") or default_last_place_path()))
        if not path.is_file():
            return observed(
                {"path": str(path)},
                False,
                "last known place unavailable",
                ["last_known_place_missing"],
            )
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            return observed(
                {"path": str(path), "error": str(exc)},
                False,
                "last known place unreadable",
                ["last_known_place_unreadable"],
            )
        data["path"] = str(path)
        return ok(data, "last known place received")


TOOL_DEFINITIONS = [
    {
        "name": "get_robot_health",
        "description": "Read aggregate AMR health. Does not command motion.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "timeout_sec": {"type": "number", "default": DEFAULT_TIMEOUT_SEC},
                "require_localization": {"type": "boolean", "default": True},
                "max_pose_age_sec": {"type": "number", "default": 3.0},
                "max_scan_age_sec": {"type": "number", "default": 1.0},
            },
        },
    },
    {
        "name": "get_safety_state",
        "description": "Read STM and safety supervisor state. Does not clear faults or reset intervention.",
        "inputSchema": {"type": "object", "properties": {"timeout_sec": {"type": "number"}}},
    },
    {
        "name": "get_localization_state",
        "description": "Read AMCL, scan, and TF localization readiness. Does not start localization.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "timeout_sec": {"type": "number"},
                "max_pose_age_sec": {"type": "number"},
                "max_scan_age_sec": {"type": "number"},
            },
        },
    },
    {
        "name": "get_mission_state",
        "description": "Read current AMR mission state from /amr_missions/state.",
        "inputSchema": {"type": "object", "properties": {"timeout_sec": {"type": "number"}}},
    },
    {
        "name": "list_named_places",
        "description": "Read named mission places from /amr_missions/list_places.",
        "inputSchema": {"type": "object", "properties": {"timeout_sec": {"type": "number"}}},
    },
    {
        "name": "get_stm_diagnostics",
        "description": "Read /amr_stm diagnostic topic cache. Does not publish to STM topics.",
        "inputSchema": {"type": "object", "properties": {"timeout_sec": {"type": "number"}}},
    },
    {
        "name": "get_navigation_state",
        "description": "Read Nav2 lifecycle states. Does not cancel or start goals.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "timeout_sec": {"type": "number"},
                "nodes": {"type": "array", "items": {"type": "string"}},
            },
        },
    },
    {
        "name": "get_last_known_place",
        "description": "Read the last named place recorded after a successful mission. Does not estimate or set robot pose.",
        "inputSchema": {"type": "object", "properties": {"path": {"type": "string"}}},
    },
]


class McpServer:
    def __init__(self) -> None:
        self.tools = AmrStateTools()
        self.tool_handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
            "get_robot_health": self.tools.get_robot_health,
            "get_safety_state": self.tools.get_safety_state,
            "get_localization_state": self.tools.get_localization_state,
            "get_mission_state": self.tools.get_mission_state,
            "list_named_places": self.tools.list_named_places,
            "get_stm_diagnostics": self.tools.get_stm_diagnostics,
            "get_navigation_state": self.tools.get_navigation_state,
            "get_last_known_place": self.tools.get_last_known_place,
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
                response = self.handle(request)
                if response is not None:
                    self.write_message(response)
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

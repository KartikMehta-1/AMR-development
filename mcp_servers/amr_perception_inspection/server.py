#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from typing import Any, Callable


SERVER_NAME = "amr-perception-inspection"
SERVER_VERSION = "0.1.0"


def ok(data: Any, message: str) -> dict[str, Any]:
    return {
        "available": True,
        "ok": True,
        "message": message,
        "data": data,
        "blockers": [],
        "warnings": [],
    }


def unavailable(data: Any, message: str, blockers: list[str]) -> dict[str, Any]:
    return {
        "available": False,
        "ok": False,
        "message": message,
        "data": data,
        "blockers": blockers,
        "warnings": [],
    }


class AmrPerceptionTools:
    def get_camera_health(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return unavailable(
            {
                "color_topic": arguments.get("color_topic", "/camera/color/image_raw"),
                "depth_topic": arguments.get("depth_topic", "/camera/depth/image_rect_raw"),
                "pointcloud_topic": arguments.get("pointcloud_topic", "/camera/depth/points"),
            },
            "camera health client not implemented yet",
            ["perception_runtime_not_connected"],
        )

    def describe_perception_contract(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return ok(
            {
                "authority": "proposal_only",
                "required_fields": ["frame_id", "timestamp_or_age", "confidence", "source"],
                "blocked_actions": [
                    "publish_cmd_vel",
                    "start_nav2_goal",
                    "execute_arm_trajectory",
                    "close_gripper",
                    "clear_faults",
                ],
                "handoff": "mission_control or future manipulation_control MCP after readiness checks and confirmation",
            },
            "perception MCP contract",
        )

    def inspect_scene(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return unavailable(
            {"max_age_sec": arguments.get("max_age_sec", 1.0)},
            "scene inspection runtime not implemented yet",
            ["scene_snapshot_unavailable"],
        )

    def list_visible_objects(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return unavailable(
            {"min_confidence": arguments.get("min_confidence", 0.5)},
            "object proposal runtime not implemented yet",
            ["object_detector_unavailable"],
        )

    def propose_grasp_candidates(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return unavailable(
            {
                "object_label": arguments.get("object_label"),
                "planning_required": True,
                "proposal_only": True,
            },
            "grasp proposal runtime not implemented yet",
            ["grasp_proposer_unavailable"],
        )


TOOL_DEFINITIONS = [
    {
        "name": "get_camera_health",
        "description": "Read RGB-D camera stream health. Does not start camera capture or command motion.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "color_topic": {"type": "string"},
                "depth_topic": {"type": "string"},
                "pointcloud_topic": {"type": "string"},
            },
        },
    },
    {
        "name": "describe_perception_contract",
        "description": "Describe proposal-only perception boundaries and blocked actions.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "inspect_scene",
        "description": "Return a structured scene summary when a perception runtime exists.",
        "inputSchema": {"type": "object", "properties": {"max_age_sec": {"type": "number"}}},
    },
    {
        "name": "list_visible_objects",
        "description": "Return visible object proposals when an object detector exists.",
        "inputSchema": {"type": "object", "properties": {"min_confidence": {"type": "number"}}},
    },
    {
        "name": "propose_grasp_candidates",
        "description": "Return grasp proposals only. Does not plan or execute arm motion.",
        "inputSchema": {"type": "object", "properties": {"object_label": {"type": "string"}}},
    },
]


class McpServer:
    def __init__(self) -> None:
        self.tools = AmrPerceptionTools()
        self.tool_handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
            "get_camera_health": self.tools.get_camera_health,
            "describe_perception_contract": self.tools.describe_perception_contract,
            "inspect_scene": self.tools.inspect_scene,
            "list_visible_objects": self.tools.list_visible_objects,
            "propose_grasp_candidates": self.tools.propose_grasp_candidates,
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
        while True:
            request = self.read_message()
            if request is None:
                break
            response = self.handle(request)
            if response is not None:
                self.write_message(response)

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

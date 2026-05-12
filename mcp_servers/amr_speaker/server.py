#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Callable


SERVER_NAME = "amr-speaker"
SERVER_VERSION = "0.1.0"
REPO_ROOT = Path(__file__).resolve().parents[2]

voice_path = REPO_ROOT / "ros_ws" / "src" / "amr_voice"
if str(voice_path) not in sys.path:
    sys.path.insert(0, str(voice_path))

from amr_voice.tts import MAX_SPEECH_CHARS, sanitize_speech_text  # noqa: E402


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
        "data": data,
        "blockers": list(blockers or []),
        "warnings": list(warnings or []),
    }


class RosSpeechPublisher:
    def __init__(self) -> None:
        self.started = False
        self.rclpy = None
        self.node = None
        self.publisher = None
        self.String = None

    def start(self) -> None:
        if self.started:
            return
        import rclpy
        from std_msgs.msg import String

        if not rclpy.ok():
            rclpy.init(args=None)
        self.rclpy = rclpy
        self.String = String
        self.node = rclpy.create_node(f"amr_speaker_mcp_{os.getpid()}")
        self.publisher = self.node.create_publisher(String, "/amr_voice/say", 10)
        self.started = True
        deadline = time.monotonic() + float(os.environ.get("AMR_MCP_GRAPH_WARMUP_SEC", "0.2"))
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def publish(self, payload: dict[str, Any]) -> None:
        self.start()
        match_deadline = time.monotonic() + float(
            os.environ.get("AMR_SPEAKER_MCP_MATCH_SEC", "1.0")
        )
        while self.publisher.get_subscription_count() == 0 and time.monotonic() < match_deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        msg = self.String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.publisher.publish(msg)
        publish_deadline = time.monotonic() + float(
            os.environ.get("AMR_SPEAKER_MCP_PUBLISH_SPIN_SEC", "0.2")
        )
        while time.monotonic() < publish_deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def shutdown(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        self.node = None
        self.publisher = None
        self.started = False
        if self.rclpy is not None and self.rclpy.ok():
            self.rclpy.shutdown()


class SpeakerTools:
    def __init__(self) -> None:
        self.ros = RosSpeechPublisher()

    def get_speaker_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        data = {
            "server": SERVER_NAME,
            "version": SERVER_VERSION,
            "speech_topic": "/amr_voice/say",
            "feedback_topic": "/amr_voice/feedback",
            "commands_motion": False,
            "clears_faults": False,
            "requires_tts_node": True,
            "max_speech_chars": MAX_SPEECH_CHARS,
            "ros_publisher_started": self.ros.started,
        }
        return response(True, "speaker MCP status", data=data)

    def speak_text(self, arguments: dict[str, Any]) -> dict[str, Any]:
        text = sanitize_speech_text(str(arguments.get("text", "")))
        source = str(arguments.get("source", "llm"))
        priority = str(arguments.get("priority", "normal"))
        interrupt = bool(arguments.get("interrupt", False))
        dry_run = bool(arguments.get("dry_run", False))
        if not text:
            return response(False, "speech request blocked", blockers=["empty_text"])
        payload = {
            "text": text,
            "source": source,
            "priority": priority,
            "interrupt": interrupt,
        }
        if dry_run:
            return response(True, "dry-run speech request accepted", data={"speech_request": payload})
        try:
            self.ros.publish(payload)
        except Exception as exc:
            return response(False, "speech publish failed", data={"speech_request": payload}, blockers=[str(exc)])
        return response(True, "speech request published", data={"speech_request": payload, "topic": "/amr_voice/say"})

    def describe_speaker_contract(self, arguments: dict[str, Any]) -> dict[str, Any]:
        data = {
            "role": "spoken feedback output only",
            "input_topic": "/amr_voice/say",
            "feedback_topic": "/amr_voice/feedback",
            "message_shape": {
                "text": "string required",
                "source": "llm|mission|safety|voice|operator",
                "priority": "normal|high",
                "interrupt": "boolean",
            },
            "policy": [
                "TTS does not command motion.",
                "LLM/MCP tools decide what failed; TTS speaks the final summary.",
                "Do not speak high-rate telemetry; speak meaningful state changes only.",
            ],
        }
        return response(True, "speaker contract", data=data)


TOOL_DEFINITIONS = [
    {
        "name": "get_speaker_status",
        "description": "Describe the AMR speaker/TTS MCP boundary. Does not command motion.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "speak_text",
        "description": "Publish a text-to-speech request to /amr_voice/say. Does not command motion.",
        "inputSchema": {
            "type": "object",
            "required": ["text"],
            "properties": {
                "text": {"type": "string"},
                "source": {"type": "string", "default": "llm"},
                "priority": {"type": "string", "default": "normal"},
                "interrupt": {"type": "boolean", "default": False},
                "dry_run": {"type": "boolean", "default": False},
            },
        },
    },
    {
        "name": "describe_speaker_contract",
        "description": "Describe how LLMs, mission tools, and status tools should request spoken feedback.",
        "inputSchema": {"type": "object", "properties": {}},
    },
]


class McpServer:
    def __init__(self) -> None:
        self.tools = SpeakerTools()
        self.tool_handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
            "get_speaker_status": self.tools.get_speaker_status,
            "speak_text": self.tools.speak_text,
            "describe_speaker_contract": self.tools.describe_speaker_contract,
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
                response_payload = self.tool_handlers[name](arguments)
                result = {
                    "content": [{"type": "text", "text": json.dumps(response_payload, sort_keys=True)}],
                    "isError": not bool(response_payload.get("ok", False)),
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
            response_message = self.handle(request)
            if response_message is not None:
                self.write_message(response_message)

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

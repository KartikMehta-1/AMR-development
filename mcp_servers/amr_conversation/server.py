#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Callable


SERVER_NAME = "amr-conversation"
SERVER_VERSION = "0.1.0"
REPO_ROOT = Path(__file__).resolve().parents[2]

for package_path in [
    REPO_ROOT / "ros_ws" / "src" / "amr_voice",
]:
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from amr_voice.conversation import DEFAULT_KNOWN_PLACES, DEFAULT_WAKE_WORD, plan_turn  # noqa: E402


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


def known_places_from(arguments: dict[str, Any]) -> list[str]:
    known_places = arguments.get("known_places")
    if known_places is None:
        return list(DEFAULT_KNOWN_PLACES)
    if not isinstance(known_places, list):
        raise ValueError("known_places must be a list of place names")
    return [str(place).strip() for place in known_places if str(place).strip()]


class ConversationTools:
    def get_conversation_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        data = {
            "server": SERVER_NAME,
            "version": SERVER_VERSION,
            "role": "stateless conversational turn planner",
            "commands_motion": False,
            "clears_faults": False,
            "starts_recovery": False,
            "speaks_directly": False,
            "speaker_tool": "amr_speaker.speak_text",
            "voice_tool": "amr_voice_interface.parse_text_intent",
        }
        return response(True, "conversation MCP status", data=data)

    def plan_conversation_turn(self, arguments: dict[str, Any]) -> dict[str, Any]:
        text = str(arguments.get("text", "")).strip()
        source = str(arguments.get("source", "text")).strip() or "text"
        wake_word = str(arguments.get("wake_word", DEFAULT_WAKE_WORD)).strip() or DEFAULT_WAKE_WORD
        require_wake_word = bool(arguments.get("require_wake_word", False))
        dry_run = bool(arguments.get("dry_run", True))
        include_speech_request = bool(arguments.get("include_speech_request", True))
        known_places = known_places_from(arguments)

        if not text:
            return response(False, "conversation turn blocked", blockers=["empty_text"])

        turn = plan_turn(
            text,
            known_places=known_places,
            wake_word=wake_word,
            require_wake_word=require_wake_word,
            dry_run=dry_run,
        )

        speaker_request = None
        if include_speech_request:
            speaker_request = {
                "server": "amr_speaker",
                "tool": "speak_text",
                "arguments": {
                    "text": turn.assistant_response,
                    "source": "conversation",
                    "priority": "normal",
                    "interrupt": False,
                    "dry_run": dry_run,
                },
                "requires_operator_confirmation": False,
            }

        data = {
            "source": source,
            "text": turn.text,
            "known_places": known_places,
            "command": turn.command,
            "assistant_response": turn.assistant_response,
            "next_tool": turn.next_tool,
            "speaker_request": speaker_request,
            "requires_confirmation": turn.requires_confirmation,
            "allowed": turn.allowed,
            "policy": [
                "This server plans one conversational turn only.",
                "Motion and runtime-changing tools still require their own MCP confirmation gates.",
                "Robot-state answers should be based on read-only state MCP results, not guesses.",
            ],
        }
        return response(turn.allowed, "conversation turn planned", data=data, blockers=turn.blockers)

    def describe_conversation_contract(self, arguments: dict[str, Any]) -> dict[str, Any]:
        data = {
            "input": "operator text or ASR transcript",
            "output": "assistant_response plus optional MCP tool plans",
            "state": "stateless for now; caller may carry conversation_id/history later",
            "safe_tooling": [
                "Use amr_state_inspection for factual robot health/debug answers.",
                "Use amr_mission_control only through its readiness and confirmation gates.",
                "Use amr_speaker.speak_text for spoken output.",
            ],
        }
        return response(True, "conversation contract", data=data)


TOOL_DEFINITIONS = [
    {
        "name": "get_conversation_status",
        "description": "Describe the AMR conversation MCP. Does not speak or command motion.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "plan_conversation_turn",
        "description": "Plan one safe conversational turn from text or an ASR transcript.",
        "inputSchema": {
            "type": "object",
            "required": ["text"],
            "properties": {
                "text": {"type": "string"},
                "source": {"type": "string", "default": "text"},
                "known_places": {"type": "array", "items": {"type": "string"}},
                "wake_word": {"type": "string", "default": DEFAULT_WAKE_WORD},
                "require_wake_word": {"type": "boolean", "default": False},
                "dry_run": {"type": "boolean", "default": True},
                "include_speech_request": {"type": "boolean", "default": True},
            },
        },
    },
    {
        "name": "describe_conversation_contract",
        "description": "Describe how conversation should coordinate voice, state, mission, and speaker MCP tools.",
        "inputSchema": {"type": "object", "properties": {}},
    },
]


class McpServer:
    def __init__(self) -> None:
        self.tools = ConversationTools()
        self.tool_handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
            "get_conversation_status": self.tools.get_conversation_status,
            "plan_conversation_turn": self.tools.plan_conversation_turn,
            "describe_conversation_contract": self.tools.describe_conversation_contract,
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

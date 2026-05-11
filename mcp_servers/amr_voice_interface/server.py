#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable


SERVER_NAME = "amr-voice-interface"
SERVER_VERSION = "0.1.0"
REPO_ROOT = Path(__file__).resolve().parents[2]

for package_path in [
    REPO_ROOT / "ros_ws" / "src" / "amr_voice",
]:
    if str(package_path) not in sys.path:
        sys.path.insert(0, str(package_path))

from amr_voice.command_parser import (  # noqa: E402
    CANCEL,
    CONFIRM,
    GO_TO,
    LIST_PLACES,
    REJECT,
    STATUS,
    UNKNOWN,
    WAKE,
    ParsedCommand,
)
from amr_voice.intent_client import VoiceIntentAdapter  # noqa: E402


DEFAULT_WAKE_WORD = os.environ.get("AMR_VOICE_WAKE_WORD", "hey jarvis")
DEFAULT_INPUT_SOURCE = os.environ.get("AMR_VOICE_INPUT_SOURCE", "text")
DEFAULT_REQUIRE_WAKE_WORD = os.environ.get("AMR_VOICE_REQUIRE_WAKE_WORD", "false").lower() in {
    "1",
    "true",
    "yes",
}
DEFAULT_KNOWN_PLACES = tuple(
    place.strip()
    for place in os.environ.get("AMR_VOICE_KNOWN_PLACES", "home,hall,kitchen,door").split(",")
    if place.strip()
)


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


def command_payload(command: ParsedCommand) -> dict[str, Any]:
    return asdict(command)


def known_places_from(arguments: dict[str, Any]) -> list[str]:
    known_places = arguments.get("known_places")
    if known_places is None:
        return list(DEFAULT_KNOWN_PLACES)
    if not isinstance(known_places, list):
        raise ValueError("known_places must be a list of place names")
    return [str(place).strip() for place in known_places if str(place).strip()]


def next_tool_for(command: ParsedCommand, *, dry_run: bool) -> dict[str, Any] | None:
    if command.action == GO_TO and command.place:
        return {
            "server": "amr_mission_control",
            "tool": "go_to_named_place",
            "arguments": {
                "place": command.place,
                "dry_run": bool(dry_run),
                "operator_confirmed_supervised": False,
            },
            "precheck_tool": {
                "server": "amr_mission_control",
                "tool": "check_go_to_readiness",
                "arguments": {"place": command.place},
            },
            "requires_operator_confirmation": True,
            "notes": [
                "Voice MCP does not start motion.",
                "Call check_go_to_readiness first.",
                "Set operator_confirmed_supervised=true only after explicit supervised confirmation.",
            ],
        }
    if command.action == CANCEL:
        return {
            "server": "amr_mission_control",
            "tool": "cancel_mission",
            "arguments": {},
            "requires_operator_confirmation": True,
            "notes": ["Cancellation changes robot runtime state; confirm with the operator first."],
        }
    if command.action == STATUS:
        return {
            "server": "amr_mission_control",
            "tool": "get_mission_state",
            "arguments": {},
            "requires_operator_confirmation": False,
        }
    if command.action == LIST_PLACES:
        return {
            "server": "amr_mission_control",
            "tool": "list_named_places",
            "arguments": {},
            "requires_operator_confirmation": False,
        }
    return None


class VoiceInterfaceTools:
    def get_voice_interface_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        data = {
            "server": SERVER_NAME,
            "version": SERVER_VERSION,
            "role": "input-agnostic voice/text intent layer",
            "supported_input_sources": [
                "text",
                "laptop_transcript",
                "jetson_transcript",
                "mobile_transcript",
            ],
            "execution_policy": {
                "starts_asr": False,
                "commands_motion": False,
                "publishes_cmd_vel": False,
                "clears_faults": False,
                "next_motion_interface": "amr_mission_control",
            },
            "defaults": {
                "wake_word": DEFAULT_WAKE_WORD,
                "require_wake_word": DEFAULT_REQUIRE_WAKE_WORD,
                "known_places": list(DEFAULT_KNOWN_PLACES),
                "input_source": DEFAULT_INPUT_SOURCE,
            },
            "future_sources": {
                "laptop": "local ASR publishes or submits transcript",
                "jetson": "local ASR submits transcript to this same intent layer",
                "mobile": "phone app/web endpoint submits transcript text to this same intent layer",
            },
        }
        return response(True, "voice interface MCP status", data=data)

    def parse_text_intent(self, arguments: dict[str, Any]) -> dict[str, Any]:
        text = str(arguments.get("text", "")).strip()
        wake_word = str(arguments.get("wake_word", DEFAULT_WAKE_WORD)).strip() or DEFAULT_WAKE_WORD
        require_wake_word = bool(arguments.get("require_wake_word", DEFAULT_REQUIRE_WAKE_WORD))
        dry_run = bool(arguments.get("dry_run", True))
        source = str(arguments.get("source", DEFAULT_INPUT_SOURCE)).strip() or DEFAULT_INPUT_SOURCE
        known_places = known_places_from(arguments)

        adapter = VoiceIntentAdapter(known_places=known_places, wake_word=wake_word)
        decision = adapter.decide(text, require_wake_word=require_wake_word)
        blockers = list(decision.blockers)
        warnings: list[str] = []
        command = decision.command

        if command.action in {CONFIRM, REJECT, WAKE}:
            warnings.append("session_state_not_persistent_in_mcp")
        if command.action == UNKNOWN:
            blockers.append("no_safe_tool_route")

        data = {
            "source": source,
            "text": text,
            "known_places": known_places,
            "wake_word": wake_word,
            "require_wake_word": require_wake_word,
            "command": command_payload(command),
            "requires_confirmation": decision.requires_confirmation,
            "allowed": decision.allowed and not blockers,
            "next_tool": next_tool_for(command, dry_run=dry_run),
        }
        ok_value = command.action != UNKNOWN and not blockers
        message = "voice/text intent parsed" if ok_value else "voice/text intent blocked"
        return response(ok_value, message, data=data, blockers=sorted(set(blockers)), warnings=warnings)

    def describe_voice_source_contract(self, arguments: dict[str, Any]) -> dict[str, Any]:
        source = str(arguments.get("source", "laptop")).strip() or "laptop"
        data = {
            "source": source,
            "contract": {
                "producer_output": "plain transcript text",
                "required_fields": ["text"],
                "recommended_fields": ["source", "wake_word", "known_places"],
                "transport": "MCP stdio now; later HTTP/WebSocket bridge can call the same parser boundary",
                "does_audio_capture": False,
            },
            "examples": [
                {
                    "text": "hey jarvis go to kitchen",
                    "source": source,
                    "dry_run": True,
                },
                {
                    "text": "status",
                    "source": source,
                    "dry_run": True,
                },
            ],
            "safety": [
                "This MCP accepts transcripts only.",
                "Motion-causing intents require mission-control readiness and operator confirmation.",
                "ASR, wake-word, and VAD can run on laptop, Jetson, or mobile before submitting text here.",
            ],
        }
        return response(True, "voice source transcript contract", data=data)


TOOL_DEFINITIONS = [
    {
        "name": "get_voice_interface_status",
        "description": "Describe the AMR voice/text MCP boundary. Does not start ASR or command motion.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "parse_text_intent",
        "description": "Parse typed text or ASR transcript into a safe AMR intent and recommended next MCP tool. Does not command motion.",
        "inputSchema": {
            "type": "object",
            "required": ["text"],
            "properties": {
                "text": {"type": "string"},
                "source": {
                    "type": "string",
                    "description": "Input source label such as text, laptop_transcript, jetson_transcript, or mobile_transcript.",
                    "default": DEFAULT_INPUT_SOURCE,
                },
                "known_places": {
                    "type": "array",
                    "items": {"type": "string"},
                    "default": list(DEFAULT_KNOWN_PLACES),
                },
                "wake_word": {"type": "string", "default": DEFAULT_WAKE_WORD},
                "require_wake_word": {"type": "boolean", "default": DEFAULT_REQUIRE_WAKE_WORD},
                "dry_run": {
                    "type": "boolean",
                    "description": "Controls the recommended mission-control call arguments only. This MCP never starts motion.",
                    "default": True,
                },
            },
        },
    },
    {
        "name": "describe_voice_source_contract",
        "description": "Describe how laptop, Jetson, or mobile ASR producers should submit transcripts to this MCP boundary.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "source": {"type": "string", "default": "laptop"},
            },
        },
    },
]


class McpServer:
    def __init__(self) -> None:
        self.tools = VoiceInterfaceTools()
        self.tool_handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
            "get_voice_interface_status": self.tools.get_voice_interface_status,
            "parse_text_intent": self.tools.parse_text_intent,
            "describe_voice_source_contract": self.tools.describe_voice_source_contract,
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

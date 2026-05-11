#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
from dataclasses import asdict
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

from amr_voice.command_parser import (  # noqa: E402
    CANCEL,
    CONFIRM,
    DIAGNOSE,
    GO_TO,
    LIST_PLACES,
    REJECT,
    STATUS,
    UNKNOWN,
    WAKE,
)
from amr_voice.intent_client import VoiceIntentAdapter  # noqa: E402


DEFAULT_WAKE_WORD = os.environ.get("AMR_VOICE_WAKE_WORD", "hey jarvis")
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


def known_places_from(arguments: dict[str, Any]) -> list[str]:
    known_places = arguments.get("known_places")
    if known_places is None:
        return list(DEFAULT_KNOWN_PLACES)
    if not isinstance(known_places, list):
        raise ValueError("known_places must be a list of place names")
    return [str(place).strip() for place in known_places if str(place).strip()]


def normalized_text(text: str) -> str:
    return " ".join(str(text).lower().strip().split())


def next_tool_for(action: str, place: str | None, *, dry_run: bool) -> dict[str, Any] | None:
    if action == GO_TO and place:
        return {
            "server": "amr_mission_control",
            "tool": "go_to_named_place",
            "arguments": {
                "place": place,
                "dry_run": bool(dry_run),
                "operator_confirmed_supervised": False,
            },
            "precheck_tool": {
                "server": "amr_mission_control",
                "tool": "check_go_to_readiness",
                "arguments": {"place": place},
            },
            "requires_operator_confirmation": True,
            "notes": [
                "Conversation MCP does not start motion.",
                "Call check_go_to_readiness first.",
                "Set operator_confirmed_supervised=true only after explicit supervised confirmation.",
            ],
        }
    if action == CANCEL:
        return {
            "server": "amr_mission_control",
            "tool": "cancel_mission",
            "arguments": {},
            "requires_operator_confirmation": True,
            "notes": ["Cancellation changes robot runtime state; confirm with the operator first."],
        }
    if action == STATUS:
        return {
            "server": "amr_mission_control",
            "tool": "get_mission_state",
            "arguments": {},
            "requires_operator_confirmation": False,
        }
    if action == LIST_PLACES:
        return {
            "server": "amr_mission_control",
            "tool": "list_named_places",
            "arguments": {},
            "requires_operator_confirmation": False,
        }
    if action == DIAGNOSE:
        return {
            "server": "amr_state_inspection",
            "tool_plan": [
                {"tool": "get_robot_health", "arguments": {"require_localization": True}},
                {"tool": "get_safety_state", "arguments": {}},
                {"tool": "get_localization_state", "arguments": {}},
                {"tool": "get_mission_state", "arguments": {}},
                {"tool": "get_navigation_state", "arguments": {}},
                {"tool": "get_stm_diagnostics", "arguments": {}},
            ],
            "requires_operator_confirmation": False,
            "notes": [
                "Read-only diagnostic plan.",
                "LLM should summarize failures and may call amr_speaker.speak_text with the summary.",
                "Do not clear faults, re-enable STM, or start recovery without separate explicit confirmation.",
            ],
        }
    return None


def conversational_response(text: str) -> str | None:
    normalized = normalized_text(text)
    greetings = {"hello", "hi", "hey", "hey robot", "hello robot", "good morning", "good evening"}
    thanks = {"thanks", "thank you", "good job", "nice work"}
    identity = {"who are you", "what are you", "what is your name", "what can you do"}
    if normalized in greetings:
        return "Hello. I am ready to help with robot status, diagnostics, and supervised missions."
    if normalized in thanks:
        return "You are welcome."
    if normalized in identity:
        return "I am the AMR voice interface. I can discuss robot state and route safe mission requests through MCP tools."
    return None


def response_for_action(action: str, place: str | None) -> str:
    if action == GO_TO and place:
        return f"I can help send the robot to {place}, but I need supervised confirmation before motion starts."
    if action == CANCEL:
        return "I can request mission cancellation after confirmation."
    if action == STATUS:
        return "I can check the current mission status."
    if action == LIST_PLACES:
        return "I can list the named places the robot knows."
    if action == DIAGNOSE:
        return "I will check robot health, safety, localization, mission, navigation, and STM diagnostics."
    if action == CONFIRM:
        return "I heard the confirmation, but I need an active pending request before taking action."
    if action == REJECT:
        return "Understood. I will not continue with the pending request."
    if action == WAKE:
        return "I am listening."
    return "I did not find a safe robot command in that. You can ask for status, diagnostics, places, or a supervised named-place mission."


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

        adapter = VoiceIntentAdapter(known_places=known_places, wake_word=wake_word)
        decision = adapter.decide(text, require_wake_word=require_wake_word)
        command = decision.command
        next_tool = next_tool_for(command.action, command.place, dry_run=dry_run)
        response_text = conversational_response(text) if command.action == UNKNOWN else None
        if response_text is None:
            response_text = response_for_action(command.action, command.place)

        blockers = list(decision.blockers)
        is_general_conversation = command.action == UNKNOWN and conversational_response(text) is not None
        if is_general_conversation:
            blockers = [blocker for blocker in blockers if blocker != "unknown_or_ambiguous_command"]
        if command.action == UNKNOWN and not is_general_conversation:
            blockers.append("no_safe_tool_route")

        speaker_request = None
        if include_speech_request:
            speaker_request = {
                "server": "amr_speaker",
                "tool": "speak_text",
                "arguments": {
                    "text": response_text,
                    "source": "conversation",
                    "priority": "normal",
                    "interrupt": False,
                    "dry_run": dry_run,
                },
                "requires_operator_confirmation": False,
            }

        data = {
            "source": source,
            "text": text,
            "known_places": known_places,
            "command": asdict(command),
            "assistant_response": response_text,
            "next_tool": next_tool,
            "speaker_request": speaker_request,
            "requires_confirmation": decision.requires_confirmation,
            "allowed": (decision.allowed or is_general_conversation) and not blockers,
            "policy": [
                "This server plans one conversational turn only.",
                "Motion and runtime-changing tools still require their own MCP confirmation gates.",
                "Robot-state answers should be based on read-only state MCP results, not guesses.",
            ],
        }
        ok_value = not blockers
        return response(ok_value, "conversation turn planned", data=data, blockers=sorted(set(blockers)))

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

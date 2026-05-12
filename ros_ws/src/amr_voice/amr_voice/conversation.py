from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any, Iterable, Optional

from amr_voice.command_parser import (
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
from amr_voice.intent_client import VoiceIntentAdapter
from amr_voice.tts import sanitize_speech_text


DEFAULT_KNOWN_PLACES = ("home", "hall", "kitchen", "door")
DEFAULT_WAKE_WORD = "hey jarvis"


@dataclass(frozen=True)
class ConversationTurn:
    text: str
    assistant_response: str
    command: dict[str, Any]
    next_tool: Optional[dict[str, Any]]
    requires_confirmation: bool
    allowed: bool
    blockers: list[str]


def transcript_text(payload: str) -> str:
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return str(payload).strip()
    if isinstance(data, dict):
        return str(data.get("text", "")).strip()
    return str(payload).strip()


def normalized_text(text: str) -> str:
    return " ".join(str(text).lower().strip().split())


def general_response(text: str) -> str | None:
    normalized = normalized_text(text)
    greetings = {"hello", "hi", "hey", "hey robot", "hello robot", "good morning", "good evening"}
    thanks = {"thanks", "thank you", "good job", "nice work"}
    identity = {"who are you", "what are you", "what is your name", "what can you do"}
    missed = {"i did not catch that", "did not catch that", "unclear"}
    if normalized in greetings:
        return "Hello. I am ready to help with robot status, diagnostics, and supervised missions."
    if normalized in thanks:
        return "You are welcome."
    if normalized in identity:
        return "I can discuss robot state and route safe mission requests through MCP tools."
    if normalized in missed:
        return "I did not catch that. Please try a short phrase like status or hello robot."
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
        }
    if action == CANCEL:
        return {
            "server": "amr_mission_control",
            "tool": "cancel_mission",
            "arguments": {},
            "requires_operator_confirmation": True,
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
        }
    return None


def plan_turn(
    text: str,
    *,
    known_places: Optional[Iterable[str]] = None,
    wake_word: str = DEFAULT_WAKE_WORD,
    require_wake_word: bool = False,
    dry_run: bool = True,
) -> ConversationTurn:
    clean_text = sanitize_speech_text(text)
    adapter = VoiceIntentAdapter(known_places=known_places or DEFAULT_KNOWN_PLACES, wake_word=wake_word)
    decision = adapter.decide(clean_text, require_wake_word=require_wake_word)
    command = decision.command
    response = general_response(clean_text) if command.action == UNKNOWN else None
    if response is None:
        response = response_for_action(command.action, command.place)
    is_general = command.action == UNKNOWN and general_response(clean_text) is not None
    blockers = list(decision.blockers)
    if is_general:
        blockers = [blocker for blocker in blockers if blocker != "unknown_or_ambiguous_command"]
    elif command.action == UNKNOWN:
        blockers.append("no_safe_tool_route")
    return ConversationTurn(
        text=clean_text,
        assistant_response=response,
        command=asdict(command),
        next_tool=next_tool_for(command.action, command.place, dry_run=dry_run),
        requires_confirmation=decision.requires_confirmation,
        allowed=(decision.allowed or is_general) and not blockers,
        blockers=sorted(set(blockers)),
    )

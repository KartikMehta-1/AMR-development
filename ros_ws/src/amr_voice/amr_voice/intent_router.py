from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Iterable, Optional

from amr_voice.local_llm import DEFAULT_QWEN_SERVER_URL
from amr_voice.tts import sanitize_speech_text


READ_STATUS = "read_status"
DIAGNOSTICS = "diagnostics"
LIST_PLACES = "list_places"
GO_TO_PLACE = "go_to_place"
CANCEL = "cancel"
CONFIRM = "confirm"
REJECT = "reject"
GENERAL_QUESTION = "general_question"
UNKNOWN = "unknown"

ALLOWED_INTENTS = {
    READ_STATUS,
    DIAGNOSTICS,
    LIST_PLACES,
    GO_TO_PLACE,
    CANCEL,
    CONFIRM,
    REJECT,
    GENERAL_QUESTION,
    UNKNOWN,
}


SYSTEM_PROMPT = """You classify one AMR robot voice transcript.
Return JSON only. Do not include markdown or prose.
The "intent" value must be exactly one token from this list:
read_status, diagnostics, list_places, go_to_place, cancel, confirm, reject, general_question, unknown.
Allowed intents:
- read_status: user asks current robot/mission/health/status/location/activity.
- diagnostics: user asks to debug, inspect failure, explain what is wrong, or run a system check.
- list_places: user asks where the robot can go or available named places.
- go_to_place: user asks the robot to move/navigate/drive/go/return to one known place.
- cancel: user asks to stop or cancel a mission.
- confirm: user confirms a pending request.
- reject: user rejects or cancels a pending request.
- general_question: user asks a non-action informational question.
- unknown: transcript is unclear or cannot be safely classified.
Motion and cancel intents require confirmation. Read-only intents do not.
Use only the supplied known_places values for place.
Examples:
Transcript "how is the robot doing" -> {"intent":"read_status","confidence":0.9,"arguments":{},"requires_confirmation":false}
Transcript "can you debug what failed" -> {"intent":"diagnostics","confidence":0.9,"arguments":{},"requires_confirmation":false}
Transcript "take the robot to kitchen" -> {"intent":"go_to_place","confidence":0.9,"arguments":{"place":"kitchen"},"requires_confirmation":true}
Transcript "what mcp tools do you have" -> {"intent":"general_question","confidence":0.85,"arguments":{},"requires_confirmation":false}
"""


@dataclass(frozen=True)
class RoutedIntent:
    intent: str
    confidence: float
    arguments: dict
    requires_confirmation: bool
    source: str
    raw: str = ""

    @property
    def known(self) -> bool:
        return self.intent not in {"unknown", ""}


class LocalIntentRouter:
    def __init__(
        self,
        server_url: str = DEFAULT_QWEN_SERVER_URL,
        model: str = "amr-qwen",
        timeout_sec: float = 30.0,
    ):
        self.server_url = server_url
        self.model = model
        self.timeout_sec = timeout_sec

    @classmethod
    def from_env(cls) -> "LocalIntentRouter":
        return cls(
            server_url=os.environ.get("AMR_QWEN_SERVER_URL", DEFAULT_QWEN_SERVER_URL),
            model=os.environ.get("AMR_QWEN_SERVER_MODEL", "amr-qwen"),
            timeout_sec=float(os.environ.get("AMR_INTENT_ROUTER_TIMEOUT_SEC", "30")),
        )

    def route(
        self,
        text: str,
        *,
        known_places: Optional[Iterable[str]] = None,
        pending_request: Optional[dict] = None,
    ) -> RoutedIntent:
        prompt = _user_prompt(text, known_places=known_places or (), pending_request=pending_request)
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT.strip()},
                {"role": "user", "content": prompt},
            ],
            "temperature": 0.0,
            "max_tokens": 140,
            "stream": False,
        }
        request = urllib.request.Request(
            self.server_url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                raw_response = response.read().decode("utf-8", errors="replace")
        except (urllib.error.URLError, TimeoutError) as exc:
            return RoutedIntent(UNKNOWN, 0.0, {}, False, "intent_router_unavailable", str(exc))
        try:
            data = json.loads(raw_response)
            content = str(data["choices"][0]["message"]["content"])
        except (KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
            return RoutedIntent(UNKNOWN, 0.0, {}, False, "intent_router_bad_response", f"{exc}: {raw_response}")
        return parse_router_json(content)


def parse_router_json(text: str) -> RoutedIntent:
    raw = text.strip()
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", raw, flags=re.DOTALL)
        if match is None:
            return RoutedIntent(UNKNOWN, 0.0, {}, False, "intent_router_parse_failed", raw)
        try:
            data = json.loads(match.group(0))
        except json.JSONDecodeError:
            return RoutedIntent(UNKNOWN, 0.0, {}, False, "intent_router_parse_failed", raw)
    if not isinstance(data, dict):
        return RoutedIntent(UNKNOWN, 0.0, {}, False, "intent_router_invalid_json", raw)
    intent = str(data.get("intent", UNKNOWN)).strip().lower()
    if intent not in ALLOWED_INTENTS:
        intent = UNKNOWN
    confidence = _clamp_float(data.get("confidence", 0.0))
    if intent != UNKNOWN and confidence == 0.0:
        confidence = 0.6
    arguments = data.get("arguments") if isinstance(data.get("arguments"), dict) else {}
    requires_confirmation = bool(data.get("requires_confirmation", intent in {GO_TO_PLACE, CANCEL}))
    if intent in {READ_STATUS, DIAGNOSTICS, LIST_PLACES, GENERAL_QUESTION, CONFIRM, REJECT, UNKNOWN}:
        requires_confirmation = False
    if intent in {GO_TO_PLACE, CANCEL}:
        requires_confirmation = True
    return RoutedIntent(intent, confidence, arguments, requires_confirmation, "intent_router", raw)


def _user_prompt(text: str, *, known_places: Iterable[str], pending_request: Optional[dict]) -> str:
    places = [str(place).strip().lower() for place in known_places if str(place).strip()]
    pending = pending_request or {}
    return (
        f"known_places={json.dumps(places)}\n"
        f"pending_request={json.dumps(pending, sort_keys=True)}\n"
        f"transcript={json.dumps(sanitize_speech_text(text))}\n"
        "Return exactly one JSON object with keys: intent, confidence, arguments, requires_confirmation.\n"
        "Do not return the intent list. Pick one token only.\n"
        "Set confidence to 0.75-1.0 when the transcript clearly matches an allowed intent.\n"
        "Set confidence below 0.45 only when the transcript is unclear.\n"
        'For this transcript, the JSON answer is:'
    )


def _clamp_float(value) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(0.0, min(1.0, number))

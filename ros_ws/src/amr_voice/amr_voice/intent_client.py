from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

from amr_voice.command_parser import GO_TO, ParsedCommand, parse_text_command


@dataclass(frozen=True)
class VoiceIntentDecision:
    command: ParsedCommand
    requires_confirmation: bool
    allowed: bool
    blockers: list[str]


class VoiceIntentAdapter:
    """Shared voice/text intent adapter used before mission clients are called."""

    def __init__(self, known_places: Optional[Iterable[str]] = None, wake_word: str = "hey jarvis"):
        self.known_places = known_places
        self.wake_word = wake_word

    def parse(self, text: str, require_wake_word: bool = False) -> ParsedCommand:
        return parse_text_command(
            text,
            known_places=self.known_places,
            wake_word=self.wake_word,
            require_wake_word=require_wake_word,
        )

    @staticmethod
    def requires_confirmation(command: ParsedCommand) -> bool:
        return command.action == GO_TO

    def decide(self, text: str, require_wake_word: bool = False) -> VoiceIntentDecision:
        command = self.parse(text, require_wake_word=require_wake_word)
        blockers: list[str] = []
        if not command.is_actionable:
            blockers.append("unknown_or_ambiguous_command")
        return VoiceIntentDecision(
            command=command,
            requires_confirmation=self.requires_confirmation(command),
            allowed=not blockers,
            blockers=blockers,
        )

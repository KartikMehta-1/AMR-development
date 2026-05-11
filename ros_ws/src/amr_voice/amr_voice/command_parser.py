import re
import string
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Set, Union


GO_TO = "go_to"
CANCEL = "cancel"
STATUS = "status"
LIST_PLACES = "list_places"
DIAGNOSE = "diagnose"
WAKE = "wake"
CONFIRM = "confirm"
REJECT = "reject"
UNKNOWN = "unknown"


@dataclass(frozen=True)
class ParsedCommand:
    action: str
    place: Optional[str] = None
    confidence: float = 0.0
    detail: str = ""
    wake_word_detected: bool = False

    @property
    def is_actionable(self) -> bool:
        return self.action != UNKNOWN


_STOP_PHRASES = {
    "stop",
    "cancel",
    "halt",
    "abort",
    "cancel mission",
    "stop mission",
    "stop moving",
    "emergency stop",
}

_STATUS_PHRASES = {
    "status",
    "what is your status",
    "where are you",
    "what are you doing",
    "mission status",
}

_LIST_PHRASES = {
    "list places",
    "show places",
    "where can you go",
    "available places",
    "locations",
}

_DIAGNOSE_PHRASES = {
    "debug",
    "debug robot",
    "debug what failed",
    "diagnose",
    "diagnose robot",
    "what failed",
    "what is wrong",
    "what went wrong",
    "why did you stop",
    "why did mission fail",
    "system check",
    "run diagnostics",
}

_CONFIRM_PHRASES = {
    "yes",
    "yeah",
    "yep",
    "confirm",
    "confirmed",
    "correct",
    "do it",
    "go ahead",
    "proceed",
}

_REJECT_PHRASES = {
    "no",
    "nope",
    "cancel that",
    "reject",
    "wrong",
    "do not",
    "dont",
    "don't",
    "never mind",
}

_GO_PREFIXES = (
    "go to",
    "go",
    "navigate to",
    "drive to",
    "move to",
    "take me to",
    "head to",
    "send robot to",
    "send lovely to",
)

_FILLER_WORDS = {
    "a",
    "an",
    "can",
    "could",
    "please",
    "the",
    "to",
    "you",
}

_PLACE_ALIASES = {
    "home": {"base", "charging", "charging station"},
    "hall": {"hallway", "living room"},
    "kitchen": {"kitchen area"},
    "door": {"front door", "main door"},
}


def parse_text_command(
    text: str,
    known_places: Optional[Iterable[str]] = None,
    wake_word: str = "hey jarvis",
    require_wake_word: bool = False,
) -> ParsedCommand:
    normalized = _normalize(text)
    if not normalized:
        return ParsedCommand(UNKNOWN, detail="empty command")

    wake_word_normalized = _normalize(wake_word)
    wake_detected = _contains_wake_word(normalized, wake_word_normalized)
    command_text = _strip_wake_word(normalized, wake_word_normalized)
    if wake_detected and not command_text:
        return ParsedCommand(
            WAKE,
            confidence=1.0,
            detail=f"wake word '{wake_word}' detected",
            wake_word_detected=True,
        )

    command_text = _drop_polite_prefix(command_text)
    known = _normalized_places(known_places)

    if _matches_any(command_text, _STOP_PHRASES):
        return ParsedCommand(CANCEL, confidence=0.95, detail="stop/cancel command", wake_word_detected=wake_detected)

    if _matches_any(command_text, _CONFIRM_PHRASES):
        return ParsedCommand(CONFIRM, confidence=0.95, detail="confirmation", wake_word_detected=wake_detected)

    if _matches_any(command_text, _REJECT_PHRASES):
        return ParsedCommand(REJECT, confidence=0.95, detail="rejection", wake_word_detected=wake_detected)

    if require_wake_word and not wake_detected:
        return ParsedCommand(
            UNKNOWN,
            detail=f"wake word '{wake_word}' not detected",
            wake_word_detected=False,
        )

    if _matches_any(command_text, _STATUS_PHRASES):
        return ParsedCommand(STATUS, confidence=0.95, detail="status command", wake_word_detected=wake_detected)

    if _matches_any(command_text, _LIST_PHRASES):
        return ParsedCommand(
            LIST_PLACES,
            confidence=0.95,
            detail="list places command",
            wake_word_detected=wake_detected,
        )

    if _matches_any(command_text, _DIAGNOSE_PHRASES):
        return ParsedCommand(
            DIAGNOSE,
            confidence=0.95,
            detail="diagnostic/debug command",
            wake_word_detected=wake_detected,
        )

    place = _match_place(command_text, known)
    if place is not None and (_looks_like_go_to(command_text, place) or command_text == place):
        return ParsedCommand(
            GO_TO,
            place=place,
            confidence=0.90,
            detail=f"go-to command for {place}",
            wake_word_detected=wake_detected,
        )

    if "return" in command_text.split() or "come" in command_text.split():
        home = _match_place("home", known)
        if home is not None and "home" in command_text.split():
            return ParsedCommand(
                GO_TO,
                place=home,
                confidence=0.90,
                detail="return-home command",
                wake_word_detected=wake_detected,
            )

    return ParsedCommand(
        UNKNOWN,
        confidence=0.0,
        detail=f"could not map text to a command: {command_text}",
        wake_word_detected=wake_detected,
    )


def _normalize(text: str) -> str:
    lowered = text.lower().strip()
    table = str.maketrans({char: " " for char in string.punctuation})
    no_punct = lowered.translate(table)
    return re.sub(r"\s+", " ", no_punct).strip()


def _strip_wake_word(text: str, wake_word: str) -> str:
    if not wake_word:
        return text
    words = text.split()
    wake_words = wake_word.split()
    if not wake_words:
        return text
    stripped = []
    index = 0
    while index < len(words):
        if words[index : index + len(wake_words)] == wake_words:
            index += len(wake_words)
            continue
        stripped.append(words[index])
        index += 1
    return " ".join(stripped)


def _contains_wake_word(text: str, wake_word: str) -> bool:
    if not wake_word:
        return False
    words = text.split()
    wake_words = wake_word.split()
    if not wake_words:
        return False
    return any(words[index : index + len(wake_words)] == wake_words for index in range(len(words)))


def _drop_polite_prefix(text: str) -> str:
    words = text.split()
    while words and words[0] in _FILLER_WORDS:
        words.pop(0)
    return " ".join(words)


def _normalized_places(known_places: Optional[Iterable[str]]) -> Set[str]:
    if known_places is None:
        return {"home", "door", "kitchen", "hall"}
    return {_normalize(place) for place in known_places if _normalize(place)}


def _matches_any(text: str, phrases: Union[Sequence[str], Set[str]]) -> bool:
    return text in phrases or any(text.startswith(f"{phrase} ") for phrase in phrases)


def _match_place(text: str, known_places: Set[str]) -> Optional[str]:
    tokens = set(text.split())
    for place in sorted(known_places, key=len, reverse=True):
        place_forms = {place}
        if not place.endswith("s"):
            place_forms.add(f"{place}s")
        if place_forms & tokens or place == text:
            return place
        aliases = _PLACE_ALIASES.get(place, set())
        if place == "home" and "home" in tokens:
            return place
        for alias in aliases:
            if alias in text:
                return place
    return None


def _looks_like_go_to(text: str, place: str) -> bool:
    if any(text.startswith(prefix) for prefix in _GO_PREFIXES):
        return True
    tokens = set(text.split())
    if place in tokens and {"go", "navigate", "drive", "move", "head"} & tokens:
        return True
    return False

from __future__ import annotations

import argparse
import sys

from amr_voice.command_parser import UNKNOWN
from amr_voice.conversation import plan_turn
from amr_voice.local_llm import qwen_responder_from_env


def main() -> int:
    parser = argparse.ArgumentParser(description="Text-only local AMR robot chat")
    parser.add_argument("text", nargs="+", help="Question or command text")
    parser.add_argument("--no-llm", action="store_true", help="Disable local LLM fallback")
    args = parser.parse_args()

    text = " ".join(args.text)
    turn = plan_turn(text)
    if turn.allowed and turn.command.get("action") != UNKNOWN:
        print(turn.assistant_response)
        return 0
    if args.no_llm:
        print(turn.assistant_response)
        return 1 if not turn.allowed else 0

    result = qwen_responder_from_env().respond(text)
    if not result.ok:
        print(f"local chat unavailable: {result.message}", file=sys.stderr)
        print(turn.assistant_response)
        return 1
    print(result.text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

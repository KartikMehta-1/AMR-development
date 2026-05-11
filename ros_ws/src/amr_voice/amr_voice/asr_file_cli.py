from __future__ import annotations

import argparse
import json
import sys

from amr_voice.asr import (
    WhisperCppConfig,
    WhisperCppTranscriber,
    build_mcp_transcript_payload,
    default_whisper_cpp_config,
)


def parse_args() -> argparse.Namespace:
    defaults = default_whisper_cpp_config()
    parser = argparse.ArgumentParser(
        description="Transcribe a WAV file with whisper.cpp and emit a voice-MCP transcript payload."
    )
    parser.add_argument("wav_path")
    parser.add_argument("--whisper-bin", default=defaults.executable)
    parser.add_argument("--model", default=defaults.model_path)
    parser.add_argument("--language", default=defaults.language)
    parser.add_argument("--threads", type=int, default=defaults.threads)
    parser.add_argument("--source", default="laptop_transcript")
    parser.add_argument("--wake-word", default="hey jarvis")
    parser.add_argument("--no-require-wake-word", dest="require_wake_word", action="store_false")
    parser.add_argument("--known-place", action="append", dest="known_places")
    parser.set_defaults(require_wake_word=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = WhisperCppConfig(
        executable=args.whisper_bin,
        model_path=args.model,
        language=args.language,
        threads=args.threads,
    )
    try:
        transcript = WhisperCppTranscriber(config).transcribe_wav(args.wav_path)
    except Exception as exc:
        print(f"asr_file_cli error: {exc}", file=sys.stderr)
        raise SystemExit(2)

    payload = build_mcp_transcript_payload(
        transcript,
        source=args.source,
        wake_word=args.wake_word,
        require_wake_word=args.require_wake_word,
        known_places=args.known_places,
    )
    print(json.dumps({"transcript": transcript.text, "mcp_arguments": payload}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

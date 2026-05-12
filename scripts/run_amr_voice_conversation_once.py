#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import wave
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
VOICE_SRC = REPO_ROOT / "ros_ws" / "src" / "amr_voice"
if str(VOICE_SRC) not in sys.path:
    sys.path.insert(0, str(VOICE_SRC))

from amr_voice.asr import VoskConfig, VoskGrammarTranscriber, default_command_grammar  # noqa: E402


def encode_mcp(message: dict[str, Any]) -> bytes:
    body = json.dumps(message).encode("utf-8")
    return f"Content-Length: {len(body)}\r\n\r\n".encode("utf-8") + body


def read_mcp_response(stream) -> dict[str, Any]:
    headers: dict[str, str] = {}
    while True:
        line = stream.readline()
        if line == b"":
            raise RuntimeError("MCP server closed stdout")
        line = line.decode("utf-8").strip()
        if not line:
            break
        key, value = line.split(":", 1)
        headers[key.lower()] = value.strip()
    return json.loads(stream.read(int(headers["content-length"])).decode("utf-8"))


def call_mcp(
    server_path: Path,
    tool_name: str,
    arguments: dict[str, Any],
    *,
    allow_tool_error: bool = False,
) -> dict[str, Any]:
    proc = subprocess.Popen(
        [sys.executable, str(server_path)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert proc.stdin is not None
    assert proc.stdout is not None
    try:
        proc.stdin.write(
            encode_mcp(
                {
                    "jsonrpc": "2.0",
                    "id": 1,
                    "method": "initialize",
                    "params": {
                        "protocolVersion": "2024-11-05",
                        "capabilities": {},
                        "clientInfo": {"name": "amr-voice-conversation-once", "version": "0.1.0"},
                    },
                }
            )
        )
        proc.stdin.write(encode_mcp({"jsonrpc": "2.0", "method": "notifications/initialized"}))
        proc.stdin.write(
            encode_mcp(
                {
                    "jsonrpc": "2.0",
                    "id": 2,
                    "method": "tools/call",
                    "params": {"name": tool_name, "arguments": arguments},
                }
            )
        )
        proc.stdin.flush()
        read_mcp_response(proc.stdout)
        response = read_mcp_response(proc.stdout)
        result = response["result"]
        payload = json.loads(result["content"][0]["text"])
        if result.get("isError") and not allow_tool_error:
            raise RuntimeError(payload)
        return payload
    finally:
        proc.kill()
        proc.wait(timeout=5)


def record_wav(path: Path, *, device: str, seconds: float, rate: int) -> None:
    command = [
        "arecord",
        "-q",
        "-D",
        device,
        "-f",
        "S16_LE",
        "-r",
        str(rate),
        "-c",
        "1",
        "-d",
        str(max(1, int(seconds))),
        str(path),
    ]
    subprocess.run(command, check=True)


def wav_level(path: Path) -> tuple[int, int]:
    with wave.open(str(path), "rb") as audio:
        frames = audio.readframes(audio.getnframes())
    if not frames:
        return 0, 0
    samples = [
        int.from_bytes(frames[index : index + 2], byteorder="little", signed=True)
        for index in range(0, len(frames) - 1, 2)
    ]
    if not samples:
        return 0, 0
    rms = int((sum(sample * sample for sample in samples) / len(samples)) ** 0.5)
    peak = max(abs(sample) for sample in samples)
    return rms, peak


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record one utterance and route it through conversation + speaker MCPs.")
    parser.add_argument("--device", default="hw:1,7")
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--rate", type=int, default=16000)
    parser.add_argument("--vosk-model", default=str(REPO_ROOT / "models" / "vosk-model-small-en-us-0.15"))
    parser.add_argument("--known-place", action="append", dest="known_places", default=["home", "hall", "kitchen", "door"])
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with tempfile.TemporaryDirectory(prefix="amr_voice_turn_") as tmp_dir:
        wav_path = Path(tmp_dir) / "utterance.wav"
        print(f"Listening for {args.seconds:.0f}s on {args.device}...", flush=True)
        record_wav(wav_path, device=args.device, seconds=args.seconds, rate=args.rate)
        rms, peak = wav_level(wav_path)
        print(f"Audio level: rms={rms} peak={peak}", flush=True)
        transcriber = VoskGrammarTranscriber(
            VoskConfig(
                model_path=args.vosk_model,
                sample_rate=args.rate,
                grammar=default_command_grammar(args.known_places),
            )
        )
        transcript = transcriber.transcribe_wav(wav_path).text
        print(f"Transcript: {transcript}", flush=True)
        if not transcript or transcript == "[unk]":
            transcript = "I did not catch that"
        conversation = call_mcp(
            REPO_ROOT / "mcp_servers" / "amr_conversation" / "server.py",
            "plan_conversation_turn",
            {
                "text": transcript,
                "source": "laptop_transcript",
                "known_places": args.known_places,
                "dry_run": args.dry_run,
                "include_speech_request": True,
            },
            allow_tool_error=True,
        )
        data = conversation["data"]
        print(f"Robot: {data['assistant_response']}", flush=True)
        speaker_request = data.get("speaker_request")
        if speaker_request is not None:
            speaker_args = dict(speaker_request["arguments"])
            speaker_args["dry_run"] = args.dry_run
            call_mcp(REPO_ROOT / "mcp_servers" / "amr_speaker" / "server.py", "speak_text", speaker_args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

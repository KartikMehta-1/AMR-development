#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


REPO_ROOT = Path(__file__).resolve().parents[1]


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


def call_mcp(server_path: Path, tool_name: str, arguments: dict[str, Any]) -> dict[str, Any]:
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
                        "clientInfo": {"name": "amr-conversation-bridge", "version": "0.1.0"},
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
        if result.get("isError"):
            raise RuntimeError(payload)
        return payload
    finally:
        proc.kill()
        proc.wait(timeout=5)


def transcript_text(raw: str) -> str:
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return raw.strip()
    if isinstance(payload, dict):
        return str(payload.get("text", "")).strip()
    return raw.strip()


class ConversationBridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(f"amr_conversation_bridge")
        self._args = args
        self._conversation_server = REPO_ROOT / "mcp_servers" / "amr_conversation" / "server.py"
        self._speaker_server = REPO_ROOT / "mcp_servers" / "amr_speaker" / "server.py"
        self.create_subscription(String, args.transcript_topic, self._transcript_cb, 10)
        self.get_logger().info(
            f"conversation bridge listening transcript_topic={args.transcript_topic} dry_run={args.dry_run}"
        )

    def _transcript_cb(self, msg: String) -> None:
        text = transcript_text(msg.data)
        if not text:
            return
        self.get_logger().info(f"transcript received: {text}")
        try:
            conversation = call_mcp(
                self._conversation_server,
                "plan_conversation_turn",
                {
                    "text": text,
                    "source": self._args.source,
                    "known_places": self._args.known_places,
                    "dry_run": self._args.dry_run,
                    "include_speech_request": True,
                },
            )
            data = conversation["data"]
            response_text = data["assistant_response"]
            self.get_logger().info(f"assistant response: {response_text}")
            speaker_request = data.get("speaker_request")
            if speaker_request is None:
                return
            speaker_arguments = dict(speaker_request["arguments"])
            speaker_arguments["dry_run"] = self._args.dry_run
            call_mcp(self._speaker_server, "speak_text", speaker_arguments)
        except Exception as exc:
            self.get_logger().error(f"conversation bridge failed: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bridge ASR transcripts through conversation and speaker MCPs.")
    parser.add_argument("--transcript-topic", default="/amr_voice/transcript")
    parser.add_argument("--source", default="laptop_transcript")
    parser.add_argument("--known-place", action="append", dest="known_places", default=["home", "hall", "kitchen", "door"])
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ConversationBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

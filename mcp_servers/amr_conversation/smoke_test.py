#!/usr/bin/env python3
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


def encode(message: dict) -> bytes:
    body = json.dumps(message).encode("utf-8")
    return f"Content-Length: {len(body)}\r\n\r\n".encode("utf-8") + body


def read_response(stream) -> dict:
    headers = {}
    while True:
        line = stream.readline()
        if line == b"":
            raise RuntimeError("server closed stdout")
        line = line.decode("utf-8").strip()
        if not line:
            break
        key, value = line.split(":", 1)
        headers[key.lower()] = value.strip()
    return json.loads(stream.read(int(headers["content-length"])).decode("utf-8"))


def main() -> int:
    server = Path(__file__).with_name("server.py")
    proc = subprocess.Popen(
        [sys.executable, str(server)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert proc.stdin is not None
    assert proc.stdout is not None
    try:
        messages = [
            {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "initialize",
                "params": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {},
                    "clientInfo": {"name": "amr-conversation-smoke-test", "version": "0.1.0"},
                },
            },
            {"jsonrpc": "2.0", "method": "notifications/initialized"},
            {"jsonrpc": "2.0", "id": 2, "method": "tools/list"},
            {
                "jsonrpc": "2.0",
                "id": 3,
                "method": "tools/call",
                "params": {
                    "name": "plan_conversation_turn",
                    "arguments": {
                        "text": "go to kitchen",
                        "known_places": ["home", "hall", "kitchen"],
                        "dry_run": True,
                    },
                },
            },
            {
                "jsonrpc": "2.0",
                "id": 4,
                "method": "tools/call",
                "params": {
                    "name": "plan_conversation_turn",
                    "arguments": {"text": "hello robot", "dry_run": True},
                },
            },
        ]
        for message in messages:
            proc.stdin.write(encode(message))
        proc.stdin.flush()

        init_response = read_response(proc.stdout)
        tools_response = read_response(proc.stdout)
        mission_response = read_response(proc.stdout)
        chat_response = read_response(proc.stdout)
        if init_response["result"]["serverInfo"]["name"] != "amr-conversation":
            raise RuntimeError("unexpected server name")
        tool_names = {tool["name"] for tool in tools_response["result"]["tools"]}
        required = {
            "get_conversation_status",
            "plan_conversation_turn",
            "describe_conversation_contract",
        }
        missing = required - tool_names
        if missing:
            raise RuntimeError(f"missing tools: {sorted(missing)}")
        mission_payload = json.loads(mission_response["result"]["content"][0]["text"])
        if mission_payload["data"]["command"]["action"] != "go_to":
            raise RuntimeError(f"unexpected mission payload: {mission_payload}")
        if not mission_payload["data"]["requires_confirmation"]:
            raise RuntimeError(f"mission request did not require confirmation: {mission_payload}")
        chat_payload = json.loads(chat_response["result"]["content"][0]["text"])
        if chat_payload["data"]["command"]["action"] != "unknown":
            raise RuntimeError(f"unexpected chat payload: {chat_payload}")
        if "ready to help" not in chat_payload["data"]["assistant_response"]:
            raise RuntimeError(f"unexpected chat response: {chat_payload}")
        print("AMR conversation MCP smoke test: PASS")
        print(f"tools: {sorted(tool_names)}")
        return 0
    finally:
        proc.kill()
        proc.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())

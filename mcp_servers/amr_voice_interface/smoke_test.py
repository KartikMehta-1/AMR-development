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
        proc.stdin.write(
            encode(
                {
                    "jsonrpc": "2.0",
                    "id": 1,
                    "method": "initialize",
                    "params": {
                        "protocolVersion": "2024-11-05",
                        "capabilities": {},
                        "clientInfo": {"name": "amr-voice-interface-smoke-test", "version": "0.1.0"},
                    },
                }
            )
        )
        proc.stdin.write(encode({"jsonrpc": "2.0", "method": "notifications/initialized"}))
        proc.stdin.write(encode({"jsonrpc": "2.0", "id": 2, "method": "tools/list"}))
        proc.stdin.write(
            encode(
                {
                    "jsonrpc": "2.0",
                    "id": 3,
                    "method": "tools/call",
                    "params": {
                        "name": "parse_text_intent",
                        "arguments": {
                            "text": "hey jarvis go to kitchen",
                            "source": "laptop_transcript",
                            "known_places": ["home", "hall", "kitchen"],
                            "wake_word": "hey jarvis",
                            "require_wake_word": True,
                            "dry_run": True,
                        },
                    },
                }
            )
        )
        proc.stdin.write(
            encode(
                {
                    "jsonrpc": "2.0",
                    "id": 4,
                    "method": "tools/call",
                    "params": {
                        "name": "parse_text_intent",
                        "arguments": {
                            "text": "hey jarvis debug what failed",
                            "source": "laptop_transcript",
                            "known_places": ["home", "hall", "kitchen"],
                            "wake_word": "hey jarvis",
                            "require_wake_word": True,
                            "dry_run": True,
                        },
                    },
                }
            )
        )
        proc.stdin.flush()
        init_response = read_response(proc.stdout)
        tools_response = read_response(proc.stdout)
        parse_response = read_response(proc.stdout)
        diagnose_response = read_response(proc.stdout)
        if init_response["result"]["serverInfo"]["name"] != "amr-voice-interface":
            raise RuntimeError("unexpected server name")
        tool_names = {tool["name"] for tool in tools_response["result"]["tools"]}
        required = {
            "get_voice_interface_status",
            "parse_text_intent",
            "describe_voice_source_contract",
        }
        missing = required - tool_names
        if missing:
            raise RuntimeError(f"missing tools: {sorted(missing)}")
        if parse_response["result"]["isError"] is not False:
            raise RuntimeError("go_to text unexpectedly failed")
        payload = json.loads(parse_response["result"]["content"][0]["text"])
        command = payload["data"]["command"]
        next_tool = payload["data"]["next_tool"]
        if command["action"] != "go_to" or command["place"] != "kitchen":
            raise RuntimeError(f"unexpected command payload: {command}")
        if next_tool["tool"] != "go_to_named_place" or not next_tool["requires_operator_confirmation"]:
            raise RuntimeError(f"unexpected next tool: {next_tool}")
        if diagnose_response["result"]["isError"] is not False:
            raise RuntimeError("diagnose text unexpectedly failed")
        diagnose_payload = json.loads(diagnose_response["result"]["content"][0]["text"])
        diagnose_command = diagnose_payload["data"]["command"]
        diagnose_next_tool = diagnose_payload["data"]["next_tool"]
        if diagnose_command["action"] != "diagnose":
            raise RuntimeError(f"unexpected diagnose command: {diagnose_command}")
        if diagnose_next_tool["server"] != "amr_state_inspection":
            raise RuntimeError(f"unexpected diagnose tool plan: {diagnose_next_tool}")
        tool_plan = {step["tool"] for step in diagnose_next_tool["tool_plan"]}
        if "get_robot_health" not in tool_plan or "get_safety_state" not in tool_plan:
            raise RuntimeError(f"incomplete diagnose tool plan: {diagnose_next_tool}")
        print("AMR voice-interface MCP smoke test: PASS")
        print(f"tools: {sorted(tool_names)}")
        return 0
    finally:
        proc.kill()
        proc.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())

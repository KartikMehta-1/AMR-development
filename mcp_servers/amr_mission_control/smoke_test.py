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
                        "clientInfo": {"name": "amr-mission-control-smoke-test", "version": "0.1.0"},
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
                        "name": "go_to_named_place",
                        "arguments": {"place": "kitchen", "dry_run": True, "timeout_sec": 0.1},
                    },
                }
            )
        )
        proc.stdin.flush()
        init_response = read_response(proc.stdout)
        tools_response = read_response(proc.stdout)
        unavailable_response = read_response(proc.stdout)
        tool_names = {tool["name"] for tool in tools_response["result"]["tools"]}
        required = {
            "list_named_places",
            "get_mission_state",
            "check_go_to_readiness",
            "go_to_named_place",
            "cancel_mission",
        }
        if init_response["result"]["serverInfo"]["name"] != "amr-mission-control":
            raise RuntimeError("unexpected server name")
        missing = required - tool_names
        if missing:
            raise RuntimeError(f"missing tools: {sorted(missing)}")
        if unavailable_response["result"]["isError"] is not True:
            raise RuntimeError("missing ROS graph did not produce an MCP tool error")
        payload = json.loads(unavailable_response["result"]["content"][0]["text"])
        if payload["ok"] is not False:
            raise RuntimeError(f"unexpected readiness payload: {payload}")
        print("AMR mission-control MCP smoke test: PASS")
        print(f"tools: {sorted(tool_names)}")
        return 0
    finally:
        proc.kill()
        proc.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())

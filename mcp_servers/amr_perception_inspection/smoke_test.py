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
                        "clientInfo": {"name": "amr-perception-smoke-test", "version": "0.1.0"},
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
                    "params": {"name": "describe_perception_contract", "arguments": {}},
                }
            )
        )
        proc.stdin.flush()
        init_response = read_response(proc.stdout)
        tools_response = read_response(proc.stdout)
        contract_response = read_response(proc.stdout)
        tool_names = {tool["name"] for tool in tools_response["result"]["tools"]}
        required = {
            "get_camera_health",
            "describe_perception_contract",
            "inspect_scene",
            "list_visible_objects",
            "propose_grasp_candidates",
        }
        if init_response["result"]["serverInfo"]["name"] != "amr-perception-inspection":
            raise RuntimeError("unexpected server name")
        missing = required - tool_names
        if missing:
            raise RuntimeError(f"missing tools: {sorted(missing)}")
        payload = json.loads(contract_response["result"]["content"][0]["text"])
        if payload["data"]["authority"] != "proposal_only":
            raise RuntimeError(f"unexpected contract payload: {payload}")
        print("Perception MCP smoke test: PASS")
        print(f"tools: {sorted(tool_names)}")
        return 0
    finally:
        proc.kill()
        proc.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable


SERVER_NAME = "amr-robot-launch"
SERVER_VERSION = "0.1.0"
DEFAULT_TIMEOUT_SEC = float(os.environ.get("AMR_LAUNCH_MCP_TIMEOUT_SEC", "180.0"))
DEFAULT_JETSON_HOST = os.environ.get("JETSON_HOST", "jetson")
DEFAULT_DEVPC_CONTAINER = os.environ.get("AMR_DEVPC_CONTAINER", "amr_devpc")
DEFAULT_JETSON_CONTAINER = os.environ.get("AMR_JETSON_CONTAINER", "amr_foxy")
DEFAULT_SESSION_NAME = os.environ.get("AMR_DEVPC_SESSION", "amr_devpc_nav")
REPO_ROOT = Path(__file__).resolve().parents[2]
LAUNCH_SCRIPT = REPO_ROOT / "scripts" / "open_amr_devpc_navigation.sh"


def payload(
    ok_value: bool,
    message: str,
    data: Any = None,
    blockers: list[str] | None = None,
    warnings: list[str] | None = None,
) -> dict[str, Any]:
    return {
        "available": True,
        "ok": bool(ok_value),
        "message": message,
        "data": data,
        "blockers": list(blockers or []),
        "warnings": list(warnings or []),
    }


def run_command(args: list[str], timeout_sec: float = 5.0) -> dict[str, Any]:
    started = time.monotonic()
    try:
        completed = subprocess.run(
            args,
            cwd=str(REPO_ROOT),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_sec,
            check=False,
        )
        return {
            "command": args,
            "returncode": completed.returncode,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
            "duration_sec": round(time.monotonic() - started, 3),
            "timed_out": False,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "command": args,
            "returncode": None,
            "stdout": (exc.stdout or "").strip() if isinstance(exc.stdout, str) else "",
            "stderr": (exc.stderr or "").strip() if isinstance(exc.stderr, str) else "",
            "duration_sec": round(time.monotonic() - started, 3),
            "timed_out": True,
        }


def resolve_map(argument: str) -> tuple[str | None, str | None]:
    if not argument:
        return None, "map_required"
    if argument.startswith("/"):
        if not argument.startswith("/workspaces/AMR-development/"):
            return None, "absolute_map_must_be_under_workspace"
        host_path = REPO_ROOT / argument.removeprefix("/workspaces/AMR-development/")
        return argument if host_path.is_file() else None, "map_not_found"
    map_file = argument if argument.endswith(".yaml") else f"{argument}.yaml"
    host_path = REPO_ROOT / "ros_ws" / "maps" / map_file
    return argument if host_path.is_file() else None, "map_not_found"


class LaunchTools:
    def get_launch_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        timeout_sec = float(arguments.get("timeout_sec", 3.0))
        jetson_host = str(arguments.get("jetson_host", DEFAULT_JETSON_HOST))
        devpc_container = str(arguments.get("devpc_container", DEFAULT_DEVPC_CONTAINER))
        jetson_container = str(arguments.get("jetson_container", DEFAULT_JETSON_CONTAINER))
        session_name = str(arguments.get("session_name", DEFAULT_SESSION_NAME))

        data = {
            "host_commands": {cmd: shutil.which(cmd) is not None for cmd in ["docker", "tmux", "ssh", "xhost"]},
            "devpc_container": run_command(
                ["docker", "inspect", "-f", "{{.State.Status}}", devpc_container],
                timeout_sec=timeout_sec,
            ) if shutil.which("docker") else None,
            "tmux_session": run_command(
                ["tmux", "has-session", "-t", session_name],
                timeout_sec=timeout_sec,
            ) if shutil.which("tmux") else None,
            "jetson_container": run_command(
                ["ssh", jetson_host, "docker", "inspect", "-f", "{{.State.Status}}", jetson_container],
                timeout_sec=timeout_sec,
            ) if shutil.which("ssh") else None,
        }
        return payload(True, "launch status snapshot", data=data)

    def preflight_launch(self, arguments: dict[str, Any]) -> dict[str, Any]:
        map_arg = str(arguments.get("map", "")).strip()
        timeout_sec = float(arguments.get("timeout_sec", 3.0))
        jetson_host = str(arguments.get("jetson_host", DEFAULT_JETSON_HOST))
        blockers: list[str] = []
        warnings: list[str] = []

        resolved_map, map_error = resolve_map(map_arg)
        if resolved_map is None:
            blockers.append(map_error or "map_invalid")

        missing = [cmd for cmd in ["docker", "tmux", "ssh", "xhost"] if shutil.which(cmd) is None]
        blockers.extend(f"missing_command:{cmd}" for cmd in missing)

        script_ok = LAUNCH_SCRIPT.is_file() and os.access(LAUNCH_SCRIPT, os.X_OK)
        if not script_ok:
            blockers.append("launch_script_missing_or_not_executable")

        ssh_probe = None
        if shutil.which("ssh"):
            ssh_probe = run_command(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", jetson_host, "true"], timeout_sec=timeout_sec)
            if ssh_probe["returncode"] != 0:
                blockers.append("jetson_ssh_unavailable")

        data = {
            "map": map_arg,
            "resolved_map": resolved_map,
            "launch_script": str(LAUNCH_SCRIPT),
            "jetson_host": jetson_host,
            "ssh_probe": ssh_probe,
            "status": self.get_launch_status(arguments)["data"],
        }
        if blockers:
            return payload(False, "launch preflight blocked", data=data, blockers=sorted(set(blockers)), warnings=warnings)
        return payload(True, "launch preflight passed", data=data, warnings=warnings)

    def launch_navigation_stack(self, arguments: dict[str, Any]) -> dict[str, Any]:
        dry_run = bool(arguments.get("dry_run", True))
        operator_confirmed = bool(arguments.get("operator_confirmed_supervised", False))
        timeout_sec = float(arguments.get("timeout_sec", DEFAULT_TIMEOUT_SEC))
        recreate_session = bool(arguments.get("recreate_session", True))
        start_rviz = bool(arguments.get("start_rviz", True))
        map_arg = str(arguments.get("map", "")).strip()

        preflight = self.preflight_launch(arguments)
        command = [str(LAUNCH_SCRIPT), map_arg]
        env_overrides = {
            "AMR_ATTACH_TMUX": "false",
            "AMR_RECREATE_SESSION": "1" if recreate_session else "0",
            "AMR_START_RVIZ": "true" if start_rviz else "false",
        }
        preflight["data"]["command"] = command
        preflight["data"]["env_overrides"] = env_overrides
        if dry_run or not preflight["ok"]:
            return preflight
        if not operator_confirmed:
            return payload(
                False,
                "operator_confirmed_supervised must be true before launching hardware-facing runtime",
                data=preflight["data"],
                blockers=["operator_confirmation_required"],
            )

        env = os.environ.copy()
        env.update(env_overrides)
        started = time.monotonic()
        try:
            completed = subprocess.run(
                command,
                cwd=str(REPO_ROOT),
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_sec,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            result = {
                "command": command,
                "env_overrides": env_overrides,
                "returncode": None,
                "stdout": (exc.stdout or "").strip() if isinstance(exc.stdout, str) else "",
                "stderr": (exc.stderr or "").strip() if isinstance(exc.stderr, str) else "",
                "duration_sec": round(time.monotonic() - started, 3),
                "status_after": self.get_launch_status(arguments)["data"],
            }
            return payload(False, "navigation launch command timed out", data=result, blockers=["launch_command_timed_out"])
        result = {
            "command": command,
            "env_overrides": env_overrides,
            "returncode": completed.returncode,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
            "duration_sec": round(time.monotonic() - started, 3),
            "status_after": self.get_launch_status(arguments)["data"],
        }
        if completed.returncode != 0:
            return payload(False, "navigation launch command failed", data=result, blockers=["launch_command_failed"])
        return payload(True, "navigation launch command completed", data=result)


TOOL_DEFINITIONS = [
    {
        "name": "get_launch_status",
        "description": "Inspect Docker/tmux launch state. Does not start or stop robot processes.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "timeout_sec": {"type": "number", "default": 3.0},
                "jetson_host": {"type": "string", "default": DEFAULT_JETSON_HOST},
                "devpc_container": {"type": "string", "default": DEFAULT_DEVPC_CONTAINER},
                "jetson_container": {"type": "string", "default": DEFAULT_JETSON_CONTAINER},
                "session_name": {"type": "string", "default": DEFAULT_SESSION_NAME},
            },
        },
    },
    {
        "name": "preflight_launch",
        "description": "Check whether the standard navigation launcher is callable. Does not start hardware.",
        "inputSchema": {
            "type": "object",
            "required": ["map"],
            "properties": {
                "map": {"type": "string"},
                "timeout_sec": {"type": "number", "default": 3.0},
                "jetson_host": {"type": "string", "default": DEFAULT_JETSON_HOST},
            },
        },
    },
    {
        "name": "launch_navigation_stack",
        "description": "Run the standard AMR navigation launcher after preflight. Hardware-facing; live run requires supervised confirmation.",
        "inputSchema": {
            "type": "object",
            "required": ["map", "operator_confirmed_supervised"],
            "properties": {
                "map": {"type": "string"},
                "operator_confirmed_supervised": {"type": "boolean", "default": False},
                "dry_run": {"type": "boolean", "default": True},
                "recreate_session": {"type": "boolean", "default": True},
                "start_rviz": {"type": "boolean", "default": True},
                "timeout_sec": {"type": "number", "default": DEFAULT_TIMEOUT_SEC},
                "jetson_host": {"type": "string", "default": DEFAULT_JETSON_HOST},
            },
        },
    },
]


class McpServer:
    def __init__(self) -> None:
        self.tools = LaunchTools()
        self.tool_handlers: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
            "get_launch_status": self.tools.get_launch_status,
            "preflight_launch": self.tools.preflight_launch,
            "launch_navigation_stack": self.tools.launch_navigation_stack,
        }

    def handle(self, request: dict[str, Any]) -> dict[str, Any] | None:
        method = request.get("method")
        request_id = request.get("id")
        if method == "notifications/initialized":
            return None
        try:
            if method == "initialize":
                result = {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
                }
            elif method == "tools/list":
                result = {"tools": TOOL_DEFINITIONS}
            elif method == "tools/call":
                params = request.get("params") or {}
                name = params.get("name")
                arguments = params.get("arguments") or {}
                if name not in self.tool_handlers:
                    raise ValueError(f"unknown tool: {name}")
                response_payload = self.tool_handlers[name](arguments)
                result = {
                    "content": [{"type": "text", "text": json.dumps(response_payload, sort_keys=True)}],
                    "isError": not bool(response_payload.get("ok", False)),
                }
            else:
                return self.error(request_id, -32601, f"method not found: {method}")
            return {"jsonrpc": "2.0", "id": request_id, "result": result}
        except Exception as exc:
            return self.error(request_id, -32000, str(exc))

    @staticmethod
    def error(request_id: Any, code: int, message: str) -> dict[str, Any]:
        return {"jsonrpc": "2.0", "id": request_id, "error": {"code": code, "message": message}}

    def serve(self) -> None:
        while True:
            request = self.read_message()
            if request is None:
                break
            response = self.handle(request)
            if response is not None:
                self.write_message(response)

    @staticmethod
    def read_message() -> dict[str, Any] | None:
        headers: dict[str, str] = {}
        while True:
            line = sys.stdin.buffer.readline()
            if line == b"":
                return None
            line = line.decode("utf-8").strip()
            if not line:
                break
            key, value = line.split(":", 1)
            headers[key.lower()] = value.strip()
        length = int(headers.get("content-length", "0"))
        if length <= 0:
            return None
        return json.loads(sys.stdin.buffer.read(length).decode("utf-8"))

    @staticmethod
    def write_message(message: dict[str, Any]) -> None:
        body = json.dumps(message, separators=(",", ":")).encode("utf-8")
        sys.stdout.buffer.write(f"Content-Length: {len(body)}\r\n\r\n".encode("utf-8"))
        sys.stdout.buffer.write(body)
        sys.stdout.buffer.flush()


def main() -> None:
    McpServer().serve()


if __name__ == "__main__":
    main()

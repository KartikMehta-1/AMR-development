#!/usr/bin/env python3
"""Validate AMR agent harness definitions without running robot hardware."""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCENARIO_DIR = ROOT / "agent_harness" / "agent_behavior" / "scenarios"
CONTRACT_FILE = ROOT / "agent_harness" / "software_contracts" / "static_contracts.yaml"
TEST_PLAN_FILE = ROOT / "agent_harness" / "software_tests" / "software_test_plan.yaml"
ACCEPTANCE_FILE = ROOT / "agent_harness" / "hardware_acceptance" / "acceptance_checklist.yaml"
REPORT_TEMPLATE = ROOT / "agent_harness" / "hardware_acceptance" / "report_template.md"
AGENT_MEMORY_FILES = [
    ROOT / "AGENTS.md",
    ROOT / "docs" / "agentic" / "the-amr-guy_fast_memory.md",
    ROOT / "docs" / "agentic" / "the-amr-guy_context.md",
]
MCP_SERVERS = [
    "amr_state_inspection",
    "amr_mission_control",
    "amr_robot_launch",
    "amr_voice_interface",
    "amr_conversation",
    "amr_speaker",
    "amr_perception_inspection",
]


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def scalar(text: str, key: str) -> str | None:
    match = re.search(rf"^{re.escape(key)}:\s*['\"]?([^'\"\n]+)['\"]?\s*$", text, re.MULTILINE)
    return match.group(1).strip() if match else None


def list_values(text: str, key: str) -> list[str]:
    pattern = rf"^{re.escape(key)}:\s*\n((?:\s+- .+\n?)+)"
    match = re.search(pattern, text, re.MULTILINE)
    if not match:
        return []
    values = []
    for line in match.group(1).splitlines():
        item = line.strip()
        if item.startswith("- "):
            values.append(item[2:].strip().strip('"').strip("'"))
    return values


def fail(errors: list[str], message: str) -> None:
    errors.append(message)


def validate_scenarios(errors: list[str]) -> None:
    scenarios = sorted(SCENARIO_DIR.glob("*.yaml"))
    if not scenarios:
        fail(errors, f"no scenarios found in {SCENARIO_DIR.relative_to(ROOT)}")
        return

    required_keys = [
        "id",
        "agent",
        "skill",
        "prompt",
        "expected_behavior",
        "forbidden_commands",
        "required_sources",
        "hardware_required",
        "motion_allowed",
    ]

    for path in scenarios:
        text = read(path)
        for key in required_keys:
            if not re.search(rf"^{re.escape(key)}:", text, re.MULTILINE):
                fail(errors, f"{path.relative_to(ROOT)} missing key: {key}")

        skill = scalar(text, "skill")
        if skill:
            skill_path = ROOT / ".codex" / "skills" / skill / "SKILL.md"
            if not skill_path.exists():
                fail(errors, f"{path.relative_to(ROOT)} references missing skill {skill_path.relative_to(ROOT)}")

        expected = list_values(text, "expected_behavior")
        if not expected:
            fail(errors, f"{path.relative_to(ROOT)} has no expected_behavior entries")

        forbidden = list_values(text, "forbidden_commands")
        if not forbidden:
            fail(errors, f"{path.relative_to(ROOT)} has no forbidden_commands entries")

        for source in list_values(text, "required_sources"):
            source_path = ROOT / source
            if not source_path.exists():
                fail(errors, f"{path.relative_to(ROOT)} references missing source {source}")


def validate_file_exists(errors: list[str], path: Path) -> None:
    if not path.exists():
        fail(errors, f"missing required harness file {path.relative_to(ROOT)}")


def validate_support_files(errors: list[str]) -> None:
    for path in [*AGENT_MEMORY_FILES, CONTRACT_FILE, TEST_PLAN_FILE, ACCEPTANCE_FILE, REPORT_TEMPLATE]:
        validate_file_exists(errors, path)

    agent_text = read(ROOT / "AGENTS.md") if (ROOT / "AGENTS.md").exists() else ""
    if "the-amr-guy" not in agent_text:
        fail(errors, "AGENTS.md must name the AMR agent as the-amr-guy")
    if "the-door-guy" in agent_text:
        fail(errors, "AGENTS.md still references the-door-guy")

    for name in MCP_SERVERS:
        server_dir = ROOT / "mcp_servers" / name
        for filename in ["README.md", "server.py", "smoke_test.py"]:
            validate_file_exists(errors, server_dir / filename)

    if CONTRACT_FILE.exists():
        contract_text = read(CONTRACT_FILE)
        for source in list_values(contract_text, "required_sources"):
            source_path = ROOT / source
            if not source_path.exists():
                fail(errors, f"{CONTRACT_FILE.relative_to(ROOT)} references missing source {source}")

    if TEST_PLAN_FILE.exists():
        test_text = read(TEST_PLAN_FILE)
        for blocked in [
            "ros2 topic pub /cmd_vel",
            "ros2 topic pub /amr_stm/enable",
            "openocd",
            "docker compose up",
        ]:
            if blocked not in test_text:
                fail(errors, f"{TEST_PLAN_FILE.relative_to(ROOT)} missing blocked command: {blocked}")

    if ACCEPTANCE_FILE.exists():
        acceptance_text = read(ACCEPTANCE_FILE)
        for level in ["level_0_source_config", "level_1_read_only_live_robot", "level_2_supervised_motion"]:
            if level not in acceptance_text:
                fail(errors, f"{ACCEPTANCE_FILE.relative_to(ROOT)} missing {level}")


def main() -> int:
    errors: list[str] = []
    validate_file_exists(errors, ROOT / "agent_harness" / "README.md")
    validate_scenarios(errors)
    validate_support_files(errors)

    if errors:
        print("Harness validation: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1

    print("Harness validation: PASS")
    print(f"- scenarios: {len(list(SCENARIO_DIR.glob('*.yaml')))}")
    print("- contract/test/acceptance definitions present")
    print("- no hardware, ROS runtime, Docker runtime, or motion commands were run")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Run source-only AMR static contract checks."""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8", errors="replace")


def check_contains(errors: list[str], path: str, pattern: str) -> None:
    text = read(path)
    if pattern not in text:
        errors.append(f"{path} missing pattern: {pattern}")


def check_absent(errors: list[str], path: str, pattern: str) -> None:
    text = read(path)
    if pattern in text:
        errors.append(f"{path} still contains stale pattern: {pattern}")


def check_stm_ros_topics(errors: list[str]) -> None:
    topic_files = [
        "STM/STM_Firmware_AMR_v2/Core/Src/main.c",
        "ros_ws/src/amr_hardware/src/amr_hardware.cpp",
        "ros_ws/src/amr_description/urdf/ros2_control.xacro",
        "docs/architecture/STM_architecture.md",
        "docs/architecture/ros_stack_diagrams.md",
    ]
    required_topics = [
        "/amr_stm/wheel_cmd_left",
        "/amr_stm/wheel_cmd_right",
        "/amr_stm/wheel_state",
    ]

    for path in topic_files:
        for topic in required_topics:
            check_contains(errors, path, topic)

    stm_doc_files = [
        "docs/architecture/STM_architecture.md",
        "docs/architecture/ros_stack_diagrams.md",
        "docs/project/AMR_project.md",
    ]
    stale_stm_topics = [
        "/amr/wheel_cmd_left",
        "/amr/wheel_cmd_right",
        "/amr/wheel_state",
        "/amr/fault_mask",
        "/amr/safety_state",
        "/amr/enable",
        "/amr/estop",
        "/amr/clear_fault",
    ]
    for path in stm_doc_files:
        for stale in stale_stm_topics:
            check_absent(errors, path, stale)


def check_estop_pin(errors: list[str]) -> None:
    check_contains(errors, "STM/STM_Firmware_AMR_v2/Core/Inc/main.h", "#define ESTOP_Pin GPIO_PIN_10")
    check_contains(errors, "STM/STM_Firmware_AMR_v2/Core/Inc/main.h", "#define ESTOP_GPIO_Port GPIOB")
    check_contains(errors, "docs/hardware/pin_map.yaml", "pin: PB10")
    check_contains(errors, "docs/architecture/STM_architecture.md", "E-stop sense: PB10")
    check_contains(errors, "docs/hardware/wiring_schematic.md", "`PB10` + E-stop sense input")

    for path in ["docs/architecture/STM_architecture.md", "docs/hardware/wiring_schematic.md"]:
        check_absent(errors, path, "Estop GPIO PC7")
        check_absent(errors, path, "`PC7` + E-stop sense input")


def check_fault_mask(errors: list[str]) -> None:
    c_fault_sources = [
        "STM/STM_Firmware_AMR_v2/Core/Inc/control_state.h",
        "docs/architecture/STM_architecture.md",
    ]
    required_c_faults = [
        "CTRL_FAULT_ESTOP",
        "CTRL_FAULT_OC_LEFT",
        "CTRL_FAULT_OC_RIGHT",
        "CTRL_FAULT_STALL_LEFT",
        "CTRL_FAULT_STALL_RIGHT",
        "CTRL_FAULT_ENC_TIMEOUT_LEFT",
        "CTRL_FAULT_ENC_TIMEOUT_RIGHT",
        "CTRL_FAULT_ADC_STUCK",
    ]
    for path in c_fault_sources:
        for fault in required_c_faults:
            check_contains(errors, path, fault)

    decode_faults = [
        "ESTOP",
        "OC_LEFT",
        "OC_RIGHT",
        "STALL_LEFT",
        "STALL_RIGHT",
        "ENC_TIMEOUT_LEFT",
        "ENC_TIMEOUT_RIGHT",
        "ADC_STUCK",
    ]
    for fault in decode_faults:
        check_contains(errors, "scripts/amr_decode_faults.py", fault)


def check_skill_contracts(errors: list[str]) -> None:
    skill_dir = ROOT / ".codex" / "skills"
    for skill_file in sorted(skill_dir.glob("*/SKILL.md")):
        text = skill_file.read_text(encoding="utf-8")
        if not re.search(r"^---\nname: .+\ndescription: \".+\"\n---", text, re.MULTILINE):
            errors.append(f"{skill_file.relative_to(ROOT)} has invalid or unquoted frontmatter")


def main() -> int:
    errors: list[str] = []
    check_stm_ros_topics(errors)
    check_estop_pin(errors)
    check_fault_mask(errors)
    check_skill_contracts(errors)

    if errors:
        print("Static contract checks: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1

    print("Static contract checks: PASS")
    print("- STM/ROS topic namespace contracts checked")
    print("- E-stop pin documentation checked")
    print("- fault mask names checked")
    print("- skill frontmatter checked")
    print("- no hardware, ROS runtime, Docker runtime, or motion commands were run")
    return 0


if __name__ == "__main__":
    sys.exit(main())

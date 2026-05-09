---
name: amr-code-review
description: Use when reviewing AMR project code, docs, configs, scripts, firmware, ROS 2 packages, voice, perception, or manipulation changes before merge or hardware runs. Prioritizes safety regressions, robot motion risk, broken ROS/firmware contracts, missing tests, and hardware validation gaps.
---

# AMR Code Review

Use this skill when the user asks to review a diff, branch, PR, or changes before running on hardware or merging.

## Source Of Truth

Read this first:

- `docs/agentic/roles/code_review_agent.md`

Read additional contracts only when the diff touches that area:

- `docs/agentic/agent_tool_permissions.md`
- `docs/agentic/roles/stm_firmware_agent.md`
- `docs/agentic/roles/ros_core_hardware_interface_agent.md`
- `docs/agentic/roles/navigation_mission_safety_agent.md`
- `docs/agentic/roles/manipulator_moveit_agent.md`
- `docs/agentic/roles/perception_calibration_agent.md`
- `docs/agentic/roles/voice_operator_interface_agent.md`

## Workflow

1. Inspect repo state:
   - `git status --short --untracked-files=all`
   - `git diff --name-only`
   - `git diff --stat`
2. Include untracked files from `git status`; `git diff --name-only` does not show them.
3. Categorize changed files using the subsystem review matrix in `docs/agentic/roles/code_review_agent.md`.
4. Inspect the diff, untracked files, and nearby code for changed behavior.
5. Focus findings on:
   - safety regressions
   - unexpected robot motion risk
   - broken ROS topic/service/action/QoS/TF contracts
   - mission/safety timeout or cancellation regressions
   - STM firmware fault, timing, current, stale-command, or micro-ROS contract issues
   - voice confirmation or ambiguity regressions
   - stale perception/calibration/frame risks
   - missing tests for deterministic behavior
6. Do not edit files unless the user explicitly asks to fix findings.
7. Do not run hardware-facing commands.

## Output Format

Lead with findings.

```text
Findings
- [Severity] file:line - Issue. Impact. Suggested fix.

Open Questions / Assumptions
- ...

Tests / Validation Observed
- ...

Residual Risk
- ...
```

If there are no findings, say so clearly and still list untested hardware/software risk.

## Severity Guide

- High: can cause unsafe motion, weakened safety, broken fault handling, bad hardware command path, or major runtime breakage.
- Medium: can break mission/navigation/voice/perception behavior but is unlikely to directly create immediate unsafe motion.
- Low: maintainability, missing docs, or minor test gaps.

## Hard Boundaries

Flag these as high severity:

- direct raw motor/PWM control paths
- unguarded `/cmd_vel` motion paths
- safety supervisor bypass
- automatic STM re-enable
- fault clear/reset without state checks
- manipulator execution without planning and approval
- perception output used as direct actuator command

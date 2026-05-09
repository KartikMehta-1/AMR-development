---
name: amr-test-runner
description: "Use when validating AMR project changes with software-only tests, ROS 2 builds, focused package checks, firmware compile checks, or when deciding which checks are safe to run. Separates software, simulation, and hardware-required checks and must not start robot motion without explicit supervised confirmation."
---

# AMR Test Runner

Use this skill when the user asks to run tests, validate changes, check a branch, or decide what tests should be run for AMR project changes.

## Source Of Truth

Read this first:

- `docs/agentic/roles/test_runner_agent.md`

Read additional references only when relevant:

- `docs/agentic/agent_tool_permissions.md`
- `docs/agentic/roles/code_review_agent.md`
- subsystem role contracts in `docs/agentic/roles/`

## Workflow

1. Inspect changed files:
   - `git status --short --untracked-files=all`
   - `git diff --name-only`
2. Include untracked files from `git status`; `git diff --name-only` does not show them.
3. Categorize changed files using the test matrix in `docs/agentic/roles/test_runner_agent.md`.
4. Select focused software-only checks first.
5. Run safe checks only:
   - ROS package builds/tests when the ROS environment is available
   - Python unit tests or syntax/import checks
   - firmware compile checks only when the command is documented and does not flash hardware
   - agent harness definition and static contract checks
   - static inspection for launch/config/script changes when executable smoke checks are not available
6. Do not start hardware, motors, Nav2 missions, arm motion, STM reset/enable, or hardware acceptance workflows without explicit supervised confirmation.
7. Report skipped and not-runnable checks honestly.

## Common Safe Checks

Use these only when applicable and available:

```bash
colcon build --packages-select <package>
colcon test --packages-select <package>
python3 -m compileall <path>
python3 agent_harness/software/validate_harness.py
python3 agent_harness/software/run_static_contract_checks.py
```

For package selection:

- `ros_ws/src/amr_voice` -> `amr_voice`
- `ros_ws/src/amr_missions` -> `amr_missions`
- `ros_ws/src/amr_safety` -> `amr_safety`
- `ros_ws/src/amr_hardware` -> `amr_hardware`
- `ros_ws/src/amr_description` -> `amr_description`
- `ros_ws/src/amr_missions_msgs` -> `amr_missions_msgs`

## Hardware-Required Checks

Do not run these unless the user explicitly requests a supervised hardware run:

- localization readiness against live robot
- named-place mission
- safety baseline
- fault clear/recovery validation
- STM flashing/reset/enable
- arm bench motion
- live camera/depth capture for calibration

## Output Format

```text
Changed Subsystems
- ...

Commands Run
- ...

Results
- Pass:
- Fail:
- Skipped:
- Not runnable:

First Actionable Failure
- ...

Hardware Checks Not Run
- ...

Recommended Next Check
- ...
```

Never imply hardware readiness from software-only checks.

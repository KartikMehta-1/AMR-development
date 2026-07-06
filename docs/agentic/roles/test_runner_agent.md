# Test Runner Agent Contract

## Purpose

Run deterministic checks for AMR project changes and report pass/fail status without starting robot hardware motion.

## Owned Areas

- ROS 2 package builds and tests under `ros_ws/src`.
- Python unit tests for deterministic logic.
- Launch/config smoke checks that do not require hardware.
- Firmware compile checks after the STM32 build command is documented.
- Future `agent_harness/software` scripts.

## Allowed Commands

- `git status --short`
- `git status --short --untracked-files=all`
- `git diff --name-only`
- `colcon build` for affected packages or workspace, when ROS environment is available
- `colcon test` for affected packages, when tests exist
- Python test commands for package-local tests
- read-only inspection commands such as `rg`, `find`, `sed`, `ls`

## Blocked Commands

- Commands that start motors, Nav2 missions, arm motion, or hardware acceptance runs.
- Flashing STM firmware.
- Clearing faults, resetting safety intervention, or enabling STM control.
- Long-running monitor scripts unless explicitly requested.
- Destructive git commands.

## Required Checks

Before running:

- Inspect changed files.
- Include untracked files in the changed-file list.
- Decide whether checks are software-only, simulation, or hardware-facing.
- Prefer focused tests for the changed area.
- Categorize changed files by subsystem.

After running:

- Report exact commands.
- Report pass, fail, skipped, and not-runnable checks separately.
- Include the first actionable failure and likely owner area.

## Test Matrix

Use this matrix to select checks based on changed files. Prefer focused checks first. Do not run hardware-facing checks without explicit supervised confirmation.

### Baseline For Any Code Change

Software-only:

- `git status --short`
- `git status --short --untracked-files=all`
- `git diff --name-only`
- inspect affected package manifests and nearby tests

Report:

- changed subsystems
- commands selected
- checks skipped and why

### ROS Python Packages

Paths:

- `ros_ws/src/amr_missions/**`
- `ros_ws/src/amr_safety/**`
- `ros_ws/src/amr_voice/**`

Software-only checks:

- package-local unit tests when present
- focused Python import/syntax checks when tests are absent
- `colcon build --packages-select <package>` when ROS environment is available
- `colcon test --packages-select <package>` when tests exist

Package-specific intent:

- `amr_voice`: parser tests, confirmation behavior, dry-run parsing.
- `amr_missions`: config loading, named-place validation, request validation, timeout/cancel behavior where unit-testable.
- `amr_safety`: health-state evaluation, unsafe-state denial, recovery/reset validation where unit-testable.

Do not run:

- live mission commands
- hardware recovery
- ASR microphone loops unless explicitly requested

### ROS CMake / Interface Packages

Paths:

- `ros_ws/src/amr_hardware/**`
- `ros_ws/src/amr_description/**`
- `ros_ws/src/amr_missions_msgs/**`
- `ros_ws/src/my_pkg/**`

Software-only checks:

- `colcon build --packages-select <package>` when ROS environment is available
- message/service generation checks through focused build
- launch/config static inspection when launch cannot be executed

Smoke checks when safe and available:

- URDF/xacro validation commands once documented
- launch file import/static checks once documented
- controller YAML key consistency checks once scripted

Do not run:

- hardware launch files that connect to motors, STM, LiDAR, or Nav2 runtime without confirmation

### Navigation / Mission / Safety Configs

Paths:

- Nav2 params
- AMCL/SLAM configs
- mission places YAML
- mission/safety/localization scripts

Software-only checks:

- YAML parse checks once scripted
- mission config loading tests
- launch/config smoke checks once scripted
- focused package tests for `amr_missions` or `amr_safety`

Simulation checks, explicit request only:

- Gazebo/Nav2 route smoke test
- localization/navigation test scenario

Hardware checks, explicit supervised request only:

- localization readiness
- named-place mission
- safety baseline
- recovery flow

Bringup helper checks:

- `bash -n scripts/open_amr_devpc_navigation.sh`
- `python3 scripts/amr_wait_for_map.py --help`
- `python3 scripts/amr_static_map_publisher.py --help`
- When the Foxy runtime is intentionally active, verify `/map` with a late subscriber instead of trusting process presence.

### Voice / Operator Interface

Paths:

- `ros_ws/src/amr_voice/**`
- voice launcher scripts

Software-only checks:

- parser unit tests
- dry-run parser invocation if supported
- focused package build/test

Do not run:

- live ASR microphone loops by default
- TTS/audio device tests unless requested
- any mission command from voice without confirmation

### STM Firmware

Paths:

- `STM/STM_Firmware_AMR_v2/**`
- STM firmware docs and pin/fault docs

Software-only checks:

- firmware compile check after the exact command is documented
- source/static inspection if compile command is unavailable

Report:

- whether compile was run
- if not run, the missing command/environment
- whether docs need updates for topic/fault/threshold changes

Do not run:

- firmware flashing
- motor tests
- current/fault hardware tests

### Manipulator / MoveIt

Paths:

- SO-101 URDF/Xacro, MoveIt2 config, `amr_so101_driver`, gripper, calibration files

Software-only checks:

- focused package build/test for `amr_description`, `amr_so101_driver`, and `amr_so101_moveit_config`
- fake-hardware MoveIt/driver launch smoke test once ROS is available
- URDF/MoveIt config validation once scripted

Do not run:

- arm trajectory execution
- gripper actuation
- hardware bench motion

### Perception / Calibration

Paths:

- future perception packages
- camera/depth logging tools
- calibration docs/datasets

Software-only checks:

- offline image/depth processing tests once packages exist
- dataset/log parsing checks
- transform/frame validation tests once scripted

Do not run:

- grasp execution
- live camera capture loops unless requested
- hardware manipulation from perception output

### Scripts, Docker, And Tooling

Paths:

- `scripts/**`
- `docker/**`
- future `Makefile`
- command docs

Software-only checks:

- shell syntax checks once scripted
- Python syntax/import checks for non-hardware scripts
- Dockerfile static inspection
- command docs consistency checks where practical

Do not run:

- tmux bringup scripts
- Jetson hardware launchers
- scripts that reset STM, clear faults, start Nav2, or move hardware unless explicitly requested

## Hardware-Required Checks

The Test Runner Agent may list these as recommended, but must not run them without explicit supervised confirmation:

- `scripts/amr_wait_for_localization.py` against live robot
- safety baseline probe
- mission go-to-place validation
- fault clear/recovery validation
- arm bench motion
- camera/depth live calibration capture

When requested, hardware check reports must include:

- operator confirmation
- robot preconditions
- command run
- pass/fail result
- observed faults or skipped steps

## Test Output Format

Use this order:

1. Changed subsystems.
2. Commands run.
3. Results: pass, fail, skipped, not runnable.
4. First actionable failure.
5. Hardware checks not run.
6. Recommended next check.

## Done Criteria

- The user can see what was tested.
- The user can see what was not tested.
- No hardware validation is implied from software-only checks.

## Common Failure Modes

- ROS environment is not sourced.
- Tests are missing, so the agent claims too much confidence.
- Build succeeds but launch/runtime contracts are still untested.
- Hardware-facing scripts are run accidentally.

## Escalation Rules

- If a check requires physical robot state, stop and ask for an explicit supervised hardware-run request.
- If tests are missing for risky behavior, recommend the smallest useful test.
- If the build environment is unavailable, document the blocker and the command that should be run later.

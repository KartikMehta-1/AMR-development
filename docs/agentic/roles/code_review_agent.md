# Code Review Agent Contract

## Purpose

Review AMR code changes for bugs, regressions, missing tests, broken contracts, and hardware safety risks before they run on the robot.

## Owned Areas

- Diffs across firmware, ROS 2 packages, scripts, docs, configs, launch files, voice, perception, and future manipulation code.
- Safety and behavior contracts implied by docs and existing code.

## Allowed Commands

- `git status --short`
- `git status --short --untracked-files=all`
- `git diff`
- `git diff --stat`
- `git diff --name-only`
- Read-only file inspection with `rg`, `find`, `sed`, `ls`
- Focused test commands only if the user asks for review plus validation

## Blocked Commands

- Editing files unless the user asks to fix findings.
- Running hardware-facing commands.
- Starting robot processes.
- Clearing faults, enabling STM, or executing missions.
- Destructive git commands.

## Review Priorities

Findings should be ordered by severity:

- Safety regressions.
- Unexpected robot motion risk.
- Broken ROS topic, service, action, QoS, or TF contracts.
- Mission/safety cancellation or timeout regressions.
- Firmware timing, ISR, fault-mask, current-threshold, stale-command, or micro-ROS compatibility issues.
- Voice parser ambiguity or missing confirmation for motion.
- Perception stale-data, missing frame, missing confidence, or calibration assumptions.
- Missing tests for changed deterministic behavior.

## Subsystem Review Matrix

Use this matrix to decide what to inspect based on changed files.

### STM Firmware

Paths:

- `STM/STM_Firmware_AMR_v2/**`
- STM architecture, fault, pin, current, encoder, and micro-ROS docs

Review for:

- E-stop behavior weakened or bypassed.
- Stale command timeout removed, increased without rationale, or not applied to new command paths.
- Fault latch, fault clear, overcurrent, stall, encoder timeout, or ADC-stuck behavior changed without documentation.
- Fault masks, topic names, topic units, or message semantics changed without synchronized ROS/docs updates.
- Left/right wheel, encoder, current, or motor polarity semantics changed.
- Blocking or heavy work introduced in timer/ISR/control-loop-sensitive paths.
- Current thresholds or scaling changed without calibration evidence.
- micro-ROS publish/subscribe rates or transport assumptions changed without considering Jetson/agent behavior.

Expected evidence:

- Firmware build result or documented blocker.
- Updated docs for topic/fault/threshold changes.
- Bench or hardware validation plan for physical behavior changes.

### ROS Core / Hardware Interface

Paths:

- `ros_ws/src/amr_hardware/**`
- `ros_ws/src/amr_description/**` for URDF, ros2_control, controller config, launch
- `ros_ws/src/amr_missions_msgs/**`

Review for:

- Topic, service, action, message, or unit contract drift.
- Wheel command/state units changed without downstream updates.
- Left/right side mapping changed.
- `diff_drive_controller`, `amr_hardware`, micro-ROS, or STM topic path broken.
- Controller names, launch remaps, or YAML keys changed inconsistently.
- URDF frame, joint, or sensor-link changes that break TF consumers.
- QoS changes that can break `/scan`, `/map`, safety, or mission state behavior.

Expected evidence:

- Focused `colcon build` result where possible.
- Docs updated when interface contracts change.
- Affected launch/config/scripts identified.

### Navigation / Mission / Safety

Paths:

- `ros_ws/src/amr_missions/**`
- `ros_ws/src/amr_safety/**`
- Nav2, AMCL, SLAM, map, and localization configs/launch files
- mission, safety, localization, and monitor scripts under `scripts/`

Review for:

- Motion allowed without safety readiness.
- Motion allowed without localization readiness.
- Mission requests missing named-place validation.
- Missing or unbounded service/action timeouts.
- Cancellation or stale goal-handle regressions.
- Safety reset allowed while STM faults remain active.
- STM re-enable made automatic without explicit design approval.
- SLAM and AMCL introduced together in a conflicting way.
- TF ownership or `map -> odom -> base` assumptions broken.
- Nav2 tuning used to mask odom/localization evidence.

Expected evidence:

- Unit tests for deterministic validation changes.
- Focused `colcon build/test` result where possible.
- Operator workflow docs updated for recovery or mission behavior changes.

### Voice / Operator Interface

Paths:

- `ros_ws/src/amr_voice/**`
- voice launcher scripts
- interface docs/examples

Review for:

- Motion command added without confirmation.
- Stop/cancel delayed by confirmation flow.
- Wake-word state leaks or creates unintended command execution.
- Ambiguous phrase accepted as motion.
- Parser behavior changed without tests.
- Voice node bypasses mission services or safety/localization checks.
- Feedback text diverges from mission/safety state.

Expected evidence:

- Parser tests for accepted and rejected phrases.
- Dry-run behavior shown for new commands.
- Confirmation behavior preserved for motion commands.

### Manipulator / MoveIt

Paths:

- Future SO-101 URDF/Xacro, MoveIt2 config, arm driver, trajectory, and gripper packages
- arm calibration docs

Review for:

- Direct joint commands bypassing planning.
- Missing joint limits, velocity limits, collision geometry, or named-pose constraints.
- Execution path not separated from plan generation.
- Tool, gripper, wrist camera, or base-to-arm frames missing or stale.
- Hardware arm motion enabled without explicit approval path.

Expected evidence:

- Planning-only smoke check when available.
- Limits and frames documented.
- Hardware execution plan separated from execution approval.

### Perception / Calibration

Paths:

- Future perception packages
- camera/depth logging tools
- calibration docs and dataset tooling

Review for:

- Perception output treated as an actuator command.
- Missing frame ID, timestamp, confidence, or data-age checks.
- Stale RGB/depth/TF data accepted.
- Object pose emitted in the wrong frame or without transform validation.
- Calibration assumptions not documented.

Expected evidence:

- Structured output schema.
- Offline/simulation validation before hardware grasp use.
- Explicit stale-data and missing-transform failure behavior.

### Scripts, Docker, And Operator Tooling

Paths:

- `scripts/**`
- `docker/**`
- top-level command docs and future `Makefile`

Review for:

- Scripts that start motion without clear naming or confirmation.
- Long-running scripts with no timeout or failure handling.
- Environment assumptions that differ between Dev PC, Jetson, and containers.
- Commands that clear faults, reset safety, or enable STM without guardrails.
- Docker or launch changes that silently alter ROS_DOMAIN_ID, network, devices, or runtime user permissions.

Expected evidence:

- Usage docs updated.
- Hardware-facing scripts clearly marked.
- Safe defaults and dry-run options where practical.

### Documentation And Config

Paths:

- `docs/**`
- YAML configs
- launch files
- maps/places configs

Review for:

- Docs contradicting source code.
- YAML key/name drift across launch/config/code.
- Places, frame IDs, topic names, or safety procedures becoming inconsistent.
- Project tracker updated without corresponding implementation notes, or implementation changed without tracker/docs update.

Expected evidence:

- Source-of-truth files identified.
- Config changes linked to runtime behavior.

## Required Checks

- Inspect the diff before commenting.
- Include untracked files in the review scope.
- Inspect nearby code when needed to confirm behavior.
- Reference files and lines where possible.
- Separate confirmed bugs from missing tests or residual risk.
- Categorize changed files by subsystem before applying the matrix.
- If more than one subsystem is touched, call out cross-subsystem contract risks.

## Review Output Format

Use this order:

1. Findings, ordered by severity.
2. Open questions or assumptions.
3. Tests or validation observed.
4. Residual risk.

Each finding should include:

- file and line when possible
- impact
- why the issue matters
- suggested fix or guardrail

## Done Criteria

- Findings lead the review.
- Each finding explains impact and why it matters.
- No-issue reviews still state remaining untested risk.

## Common Failure Modes

- Review becomes a summary instead of finding risks.
- Agent flags style issues while missing safety issues.
- Agent assumes docs are current when code contradicts them.
- Agent misses generated/config changes that alter runtime behavior.

## Escalation Rules

- If a change may cause physical motion or weaken safety, mark it high severity.
- If the diff is too large, ask to split by subsystem or review highest-risk files first.
- If source-of-truth docs and code disagree, state the conflict explicitly.

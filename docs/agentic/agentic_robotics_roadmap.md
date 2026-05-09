# Agentic Robotics Roadmap

This document tracks how agent-oriented tooling should be introduced into the AMR + SO-101 manipulator project.

The goal is not to replace ROS 2, Nav2, MoveIt2, `ros2_control`, micro-ROS, or STM32 firmware. Those remain the deterministic robot control stack. Agent tooling should sit above them as a structured operator/developer interface with clear permissions, typed tools, logs, and test harnesses.

## Architecture Principle

```text
Human / voice / developer prompt
        |
Agent orchestration layer
        |
Skills + MCP tools + harnesses
        |
Typed ROS services/actions and read-only diagnostics
        |
Mission server / safety supervisor / Nav2 / MoveIt2
        |
ros2_control / micro-ROS / STM32 firmware
        |
AMR base, sensors, and SO-101 manipulators
```

Agents may inspect state, explain faults, compose missions, generate code, and request typed actions. They must not bypass the existing safety supervisor or directly drive motors through raw velocity, PWM, or joint commands.

## Implementation Sequence

Do not start by building a full multi-agent runtime. Start with static contracts and skills that improve day-to-day work immediately, then add harnesses and MCP wrappers after the boundaries are clear.

```text
1. Agent contracts and permissions
2. Repo-local skills
3. Test/review/agent harness
4. Shared ROS client libraries
5. Read-only MCP server
6. Confirmation-required MCP tools
7. Subagent workflow
8. Manipulation/perception expansion
```

## Phase 1 - Agent Contracts And Permission Model

- Add `docs/agentic/agent_tool_permissions.md` defining read-only, confirmation-required, hardware-motion, and blocked tool categories. Initial version added on 2026-05-09.
- Add `docs/agentic/agent_interaction_examples.md` with realistic prompts for navigation debugging, safety recovery, mission control, voice commands, and manipulator bring-up. Initial version added on 2026-05-09.
- Add `docs/agentic/codebase_ownership.md` mapping repo areas to primary/secondary agent ownership. Initial version added on 2026-05-09.
- Add `docs/agentic/roles/` with one contract per target subagent. Initial contracts added on 2026-05-09:
  - `test_runner_agent.md`
  - `code_review_agent.md`
  - `stm_firmware_agent.md`
  - `ros_core_hardware_interface_agent.md`
  - `navigation_mission_safety_agent.md`
  - `manipulator_moveit_agent.md`
  - `perception_calibration_agent.md`
  - `voice_operator_interface_agent.md`
  - `runtime_environment_agent.md`
- Each role contract must define:
  - purpose
  - owned files
  - allowed commands
  - blocked commands
  - required checks
  - done criteria
  - escalation rules
- Define the rule that the agent layer uses intent-level tools only. Examples: `go_to_named_place`, `cancel_mission`, `get_robot_health`, `plan_arm_named_pose`.
- Explicitly block raw tools such as direct motor PWM, direct `/cmd_vel` publishing for motion, disabling safety, or unguarded joint motion.

Target subagent structure:

- 1. Test Runner Agent
- 2. Code Review Agent
- 3. STM Firmware Agent
- 4. ROS Core / Hardware Interface Agent
- 5. Navigation / Mission / Safety Agent
- 6. Manipulator / MoveIt Agent
- 7. Perception / Calibration Agent
- 8. Voice / Operator Interface Agent
- 9. Runtime Environment Agent

Implementation priority:

- Agents 1-5 should be populated first because they map to current active work.
- Agent 9 should be populated alongside Jetson Orin NX runtime work.
- Agents 6-8 are defined now but can stay lighter until manipulator, perception, and voice/UI work expands.

## Engineering Agent Roles

These agents support development of the robot software and firmware. They are allowed to inspect and edit code when requested, but hardware-facing validation remains explicit and supervised.

### Test Runner Agent

Purpose:

- Run deterministic checks after code changes.
- Summarize failures with the command, failing package, failing test, and likely owner area.
- Distinguish software-only tests from simulation and hardware tests.

Initial tool scope:

- `colcon build`
- `colcon test`
- Python unit tests for `amr_voice`, `amr_missions`, and `amr_safety`
- launch/config smoke checks
- firmware compile checks when the STM32 build command is documented

Boundaries:

- May run software-only checks without robot hardware.
- Must not start motors, Nav2 missions, arm motion, or hardware acceptance tests unless the user explicitly requests a supervised hardware run.
- Should produce a short pass/fail report with exact next action.

Done criteria:

- Reports command results clearly.
- Points to failing files/tests.
- Does not hide skipped hardware tests.

### Code Review Agent

Purpose:

- Review code changes before they reach the robot.
- Prioritize bugs, regressions, missing safety checks, missing tests, topic/service contract breaks, and hardware-risky changes.

Review focus by area:

- STM firmware: fault handling, timing, ISR behavior, watchdog/staleness behavior, saturation, sign/polarity, current thresholds, micro-ROS topic compatibility.
- ROS 2 stack: topic names, QoS, lifecycle behavior, launch order, TF ownership, Nav2/MoveIt contract compatibility.
- Mission/safety: request validation, timeout behavior, cancellation, recovery paths, unsafe state denial.
- Voice/interface: parser determinism, confirmation behavior, wake-word gating, rejection of ambiguous commands.
- Perception/image processing: calibration assumptions, frame IDs, confidence thresholds, stale image/depth handling, deterministic fallbacks.

Boundaries:

- Review agents should not make code changes unless asked to fix findings.
- Findings should be severity-ordered and include file/line references where possible.
- If no issues are found, they should still state remaining untested risk.

Done criteria:

- Review output starts with actionable findings.
- Safety and hardware-facing risks are called out explicitly.
- Missing tests are identified separately from confirmed bugs.

### STM Firmware Code Agent

Purpose:

- Implement bounded changes in `STM/STM_Firmware_AMR_v2` and related firmware docs.
- Preserve deterministic low-level control and safety behavior.

Typical tasks:

- Add telemetry topics.
- Adjust fault masks or thresholds.
- Refactor PID/current-sensing code.
- Add proximity sensor drivers.
- Improve micro-ROS transport robustness.

Required context:

- `docs/architecture/STM_architecture.md`
- `docs/hardware/pin_map.yaml`
- `docs/safety/safety_fault_recovery.md`
- `STM/STM_Firmware_AMR_v2/Core/Src/main.c`
- relevant transport, control, and fault source files

Boundaries:

- Must not weaken e-stop, stale command timeout, overcurrent/stall handling, or fault latching without an explicit design note.
- Must keep ROS topic/message contracts synchronized with ROS 2 clients.
- Hardware validation requires operator supervision.

Done criteria:

- Firmware builds or the missing build command is documented.
- Changed fault/topic behavior is reflected in docs.
- A bench or hardware validation plan is provided when physical behavior changes.

### ROS Core / Hardware Interface Agent

Purpose:

- Maintain the ROS 2 workspace structure and the ROS-to-hardware interface layer.
- Keep `amr_hardware`, `ros2_control`, URDF/controller configs, shared messages/services, and micro-ROS topic contracts coherent.

Typical tasks:

- Modify ROS package metadata and build files.
- Maintain `amr_hardware` and wheel state/command interfaces.
- Update URDF, ros2_control, and controller configs.
- Keep shared messages/services/actions compatible across packages.
- Coordinate ROS-side topic contract changes with STM firmware.

Required context:

- `docs/architecture/ros_stack_diagrams.md`
- `ros_ws/src/amr_hardware`
- `ros_ws/src/amr_description`
- `ros_ws/src/amr_missions_msgs`
- STM topic/fault docs when interface contracts change

Boundaries:

- Must not introduce ad hoc command paths around `ros2_control` or `amr_hardware`.
- Must preserve left/right wheel semantics and topic units.
- Must not change shared contracts without updating downstream users and docs.
- Must avoid direct motion commands when a typed service/action exists.

Done criteria:

- Focused `colcon build` result is reported or blocker is documented.
- Interface changes are documented.
- Affected packages/scripts are identified.
- No parallel hardware command path is introduced.

### Navigation / Mission / Safety Agent

Purpose:

- Maintain and diagnose SLAM, localization, Nav2, named-place missions, safety supervision, recovery, and mission-facing operator workflows.

Typical tasks:

- Add or modify mission validation and sequencing.
- Diagnose Nav2, AMCL, SLAM, TF, odom, and map issues.
- Add safety-supervisor logic and recovery behavior.
- Integrate `robot_localization`.
- Maintain named places and mission scripts.

Required context:

- `docs/architecture/ros_stack_diagrams.md`
- `docs/safety/safety_fault_recovery.md`
- `docs/safety/safety_baseline.md`
- `ros_ws/src/amr_missions`
- `ros_ws/src/amr_safety`
- Nav2/SLAM/AMCL configs and launch files
- mission/safety/localization scripts under `scripts/`

Boundaries:

- Must not create a second mission/safety implementation outside the existing mission server and safety supervisor.
- Must preserve TF ownership rules.
- Must preserve cancellation, timeout, and unsafe-state denial behavior.
- Must avoid direct motion commands when a typed service/action exists.

Done criteria:

- `colcon build` or focused package build passes when available.
- Unit or smoke tests are added for deterministic logic.
- Launch/config changes are documented if operator-facing.

### Manipulator/MoveIt Agent

Purpose:

- Develop the SO-101 manipulation stack once URDF, MoveIt2, joint drivers, and calibration work become active.
- Keep manipulation execution behind planning, limits, collision checks, and explicit approval.

Typical tasks:

- Add SO-101 URDF/Xacro integration.
- Add MoveIt2 config and named poses.
- Add joint limits, gripper frames, tool frames, and collision geometry.
- Add bench-safe planning smoke tests.

Required context:

- future SO-101 URDF/MoveIt packages
- `docs/architecture/ros_stack_diagrams.md`
- camera/gripper calibration docs once created

Boundaries:

- Must not execute unplanned joint motion.
- Must not bypass joint limits, collision checks, or safety supervisor assumptions.
- Hardware execution requires operator approval.

Done criteria:

- Planning works in simulation or bench-safe mode.
- Named poses and limits are documented.
- Execution plan is separated from execution approval.

### Voice/Interface Code Agent

Purpose:

- Develop voice, text, or future UI interfaces as intent parsers over the mission/safety layer.

Typical tasks:

- Add parser intents.
- Add confirmations and rejection handling.
- Add TTS/feedback outputs.
- Add command history/logging.
- Add operator status summaries.

Required context:

- `ros_ws/src/amr_voice`
- `ros_ws/src/amr_missions`
- safety/localization precondition docs

Boundaries:

- Voice commands that cause motion require confirmation unless they are stop/cancel.
- Ambiguous commands must ask for clarification or refuse.
- The voice layer must not bypass mission runtime or safety checks.

Done criteria:

- Parser unit tests cover new phrases.
- Confirmation behavior is tested.
- Motion commands still require localization and safety readiness.

### Perception/Image-Processing Code Agent

Purpose:

- Implement perception utilities for calibration, RGB-D processing, object detection proposals, and future grasp inputs.

Typical tasks:

- Add image/depth logging tools.
- Add AprilTag or calibration checks.
- Add RGB-D object proposal pipelines.
- Add transforms from camera frame to base/arm frame.
- Prepare datasets for grasping or VLA/VLM evaluation.

Required context:

- camera mounting/calibration docs once created
- `docs/architecture/ros_stack_diagrams.md`
- future perception packages under `ros_ws/src`
- dataset/logging conventions

Boundaries:

- Perception outputs are proposals, not direct actuator commands.
- Must include frame IDs, timestamps, confidence, and stale-data checks.
- Any grasp execution must go through planning and approval.

Done criteria:

- Outputs are structured and frame-aware.
- Failure cases are explicit.
- Bench/simulation validation exists before hardware grasp attempts.

## Phase 2 - Repo-Local Skills

Create project-specific skills as repeatable playbooks for future agent sessions.

First implementation target:

```text
.codex/skills/
  amr-test-runner/
    SKILL.md   # initial version added 2026-05-09
  amr-code-review/
    SKILL.md   # initial version added 2026-05-09
  amr-stm-firmware-dev/
    SKILL.md
  amr-ros-core-hardware-dev/
    SKILL.md
  amr-navigation-mission-safety-dev/
    SKILL.md
```

Second implementation target:

```text
.codex/skills/
  amr-nav-debug/
    SKILL.md
  amr-safety-recovery/
    SKILL.md
  amr-mission-runtime/
    SKILL.md
```

Later implementation target:

```text
.codex/skills/
  amr-manipulator-bringup/
    SKILL.md
  amr-perception-dev/
    SKILL.md
  amr-hardware-acceptance/
    SKILL.md
  amr-voice-dev/
    SKILL.md
```

Planned skills:

- `.codex/skills/amr-nav-debug/SKILL.md`
  - Source files: Nav2 params, launch files, `docs/architecture/ros_stack_diagrams.md`, mission logs.
  - Tasks: inspect TF, AMCL readiness, `/scan`, `/odom`, Nav2 lifecycle, mission status.
  - Done criteria: root-cause summary plus reproducible checks.

- `.codex/skills/amr-safety-recovery/SKILL.md`
  - Source files: `amr_safety`, fault docs, safety recovery script, safety baseline logs.
  - Tasks: decode faults, inspect supervisor state, decide whether reset/re-enable is allowed.
  - Done criteria: clear operator recovery path and no hidden automatic re-enable.

- `.codex/skills/amr-mission-runtime/SKILL.md`
  - Source files: `amr_missions`, `places.yaml`, mission CLI/server, mission status messages.
  - Tasks: add mission commands, validate named places, diagnose mission failures.
  - Done criteria: parser/client/server behavior tested without requiring hardware when possible.

- `.codex/skills/amr-manipulator-bringup/SKILL.md`
  - Source files: SO-101 URDF/MoveIt files once added, arm driver configs, calibration docs.
  - Tasks: bench-safe named poses, joint limits, planning scene, collision checks, guarded execution.
  - Done criteria: sim or bench smoke test before any hardware execution.

- `.codex/skills/amr-hardware-acceptance/SKILL.md`
  - Source files: baseline probes, safety reports, mission monitor scripts, acceptance criteria.
  - Tasks: run or interpret idle, motion, localization, mission, and safety recovery checks.
  - Done criteria: timestamped pass/fail report with known limitations.

- `.codex/skills/amr-test-runner/SKILL.md`
  - Source files: package manifests, test folders, launch/config files, Docker docs, future `Makefile`.
  - Tasks: run focused software tests, summarize failures, separate hardware-required checks from software checks.
  - Done criteria: reproducible command list and pass/fail report.

- `.codex/skills/amr-code-review/SKILL.md`
  - Source files: changed files, relevant docs/contracts, nearby tests.
  - Tasks: review changes for regressions, safety risks, missing validation, broken topic/service contracts.
  - Done criteria: severity-ordered findings with file/line references.

- `.codex/skills/amr-stm-firmware-dev/SKILL.md`
  - Source files: STM firmware source, pin map, fault docs, STM architecture docs.
  - Tasks: implement bounded firmware changes, preserve safety contracts, document validation plan.
  - Done criteria: firmware build/check result and updated docs for behavior changes.

- `.codex/skills/amr-ros-core-hardware-dev/SKILL.md`
  - Source files: `amr_hardware`, `amr_description`, shared messages/services, ros2_control configs, ROS stack docs.
  - Tasks: implement ROS interface and hardware bridge changes, preserve topic/frame/controller contracts, verify build/smoke checks.
  - Done criteria: focused `colcon` build/test result and documented interface impact.

- `.codex/skills/amr-navigation-mission-safety-dev/SKILL.md`
  - Source files: `amr_missions`, `amr_safety`, Nav2/SLAM/AMCL configs, mission/safety scripts, ROS stack and safety docs.
  - Tasks: implement navigation, mission, safety, localization, and recovery changes with guardrails.
  - Done criteria: focused `colcon` build/test result.

- `.codex/skills/amr-voice-dev/SKILL.md`
  - Source files: `amr_voice`, `amr_missions`, safety/localization checks.
  - Tasks: add parser/interface behavior with confirmation and safety guards.
  - Done criteria: parser tests and dry-run behavior.

- `.codex/skills/amr-perception-dev/SKILL.md`
  - Source files: perception packages once added, calibration docs, camera/depth logs.
  - Tasks: add image/depth processing, calibration checks, object/grasp proposal outputs.
  - Done criteria: structured outputs with frame/timestamp/confidence handling.

## Phase 3 - Agent Harness

Add a harness folder for repeatable tests and agent evaluation scenarios before exposing MCP motion tools. The harness is where agents prove that their recommendations and code changes meet the project safety rules.

Planned layout:

```text
agent_harness/
  README.md
  software/
    run_unit_tests.sh
    run_ros_smoke_checks.sh
    run_firmware_build_check.sh
  simulation/
    run_nav2_sim_smoke.sh
    run_moveit_planning_smoke.sh
  hardware_acceptance/
    README.md
  scenarios/
    nav_localization_not_ready.yaml
    safety_fault_blocks_motion.yaml
    mission_go_to_place.yaml
    manipulator_plan_without_execute.yaml
    voice_confirmation_required.yaml
    perception_stale_frame_rejected.yaml
  reports/
```

Harness levels:

- Software harness: parser tests, mission validation, safety health-state logic, launch/config smoke checks.
- Simulation harness: Gazebo/Nav2 routes and MoveIt planning scenarios.
- Hardware acceptance harness: supervised checks for idle, motion, localization, mission success, and safety recovery.

Required safety scenarios:

- If localization is not ready, motion requests must be denied.
- If STM fault mask is nonzero, mission start must be denied.
- If safety supervisor is in intervention state, agent may explain recovery but must not re-enable automatically.
- If asked to move a manipulator, agent must plan first and execute only after approval.
- If asked for raw motor or PWM control, agent must refuse and propose a safe typed alternative.
- If voice parsing is ambiguous, the agent must refuse or ask for clarification.
- If image/depth data is stale or lacks a valid transform, grasp execution must be denied.

Engineering harness scenarios:

- A code-generation agent must run or state the focused tests it could not run.
- A review agent must flag missing safety prechecks on motion-causing code.
- A firmware agent must flag changed fault masks or topic contracts that are not documented.
- A ROS 2 agent must flag launch/config changes that break known TF ownership.

## Phase 4 - Shared ROS Client Libraries

Before implementing MCP tools, pull shared request logic out of scripts where duplication would otherwise appear. CLI scripts, voice nodes, and MCP tools should call the same client functions.

Target structure:

```text
ros_ws/src/amr_missions/amr_missions/
  mission_client.py
  mission_cli.py
  mission_server.py

ros_ws/src/amr_safety/amr_safety/
  safety_client.py
  safety_supervisor.py
```

Rules:

- `mission_server` owns mission behavior.
- `safety_supervisor` owns safety behavior.
- Shared clients own request/response handling.
- CLI wrappers and MCP tools stay thin.
- The MCP server must not reimplement mission or safety logic.

Done criteria:

- Existing CLI behavior still works.
- Shared client functions return structured results.
- Timeout and failure behavior is explicit.
- Voice and future MCP paths can reuse the same client logic.

## Phase 5 - Read-Only MCP Server

Start with a read-mostly MCP server. Do not expose motion tools until the permission model and safety checks are documented.

Planned server layout:

```text
mcp_servers/
  amr_robot/
    README.md
    server.py
    tools/
      health.py
      mission.py
      safety.py
      navigation.py
      manipulation.py
```

Initial read-only tools:

- `get_robot_health()`
- `get_safety_state()`
- `get_mission_status()`
- `list_named_places()`
- `get_localization_status()`
- `get_nav2_status()`
- `get_latest_fault_summary()`

Implementation rule:

```text
Shared ROS client library
        |
CLI scripts and MCP tools
        |
Mission server / safety supervisor / Nav2 / MoveIt2
```

The MCP server should wrap existing stable ROS services/actions or shared Python client functions. It should not reimplement mission or safety logic.

Done criteria:

- Tools are read-only or diagnostic only.
- Outputs are structured, not free-form terminal text.
- Failure reasons are explicit.
- The server can answer robot readiness questions without starting motion.

## Phase 6 - Confirmation-Required MCP Tools

After read-only tools work and harness scenarios exist, add tools that can change robot state.

Confirmation-required tools:

```text
go_to_named_place(place)
cancel_current_mission()
request_safety_recovery_check()
clear_fault_after_operator_confirmation()
```

Each motion-capable tool must:

- validate input
- check safety state
- check localization readiness when motion is involved
- check mission/Nav2 availability
- require explicit confirmation when motion or recovery is involved
- log the request and result
- return structured output

Example result:

```json
{
  "ok": false,
  "reason": "localization_not_ready",
  "motion_started": false
}
```

Future manipulator tools:

```text
get_arm_state(side)
plan_arm_named_pose(side, pose_name)
plan_grasp(object_id)
execute_approved_arm_plan(plan_id)
```

Manipulator execution tools must separate planning from execution approval.

## Phase 7 - Subagent Workflow

Subagents can begin as role/skill usage patterns. They do not need to start as long-running services.

Default workflow for a code task:

```text
Main agent
  -> appropriate domain agent implements the bounded change
  -> Test Runner agent runs focused checks
  -> Code Review agent reviews the final diff
  -> Main agent summarizes outcome and remaining risks
```

Default workflow for a robot behavior issue:

```text
Main agent
  -> Navigation/mission/safety diagnosis
  -> Firmware diagnosis if wheel, current, fault, or micro-ROS behavior is implicated
  -> Test or harness check where possible
  -> Main agent proposes fix or validation run
```

Rules:

- The main agent remains the coordinator.
- Subagents return bounded outputs: patch, review, test report, diagnosis, or implementation plan.
- Subagents must not independently approve robot motion.
- Hardware actions remain explicit and supervised.

## Phase 8 - Integration With Productization

Agent tooling becomes part of the productization track only after the base interfaces are stable.

Acceptance criteria:

- Agent docs exist and are linked from the project tracker.
- The first two skills exist: test runner and code review.
- The next three skills are planned: STM firmware development, ROS core/hardware development, and navigation/mission/safety development.
- At least three robot-domain support skills exist: navigation debug, safety recovery, mission runtime.
- Optional later skills exist for manipulator bring-up, voice/interface development, perception development, and hardware acceptance.
- Shared mission/safety client functions exist where MCP or voice would otherwise duplicate script logic.
- Read-only MCP tools can report health, safety, mission, places, and localization status.
- Motion-causing MCP tools require explicit confirmation and safety prechecks.
- Agent harness includes pass/fail scenarios for unsafe motion requests.
- Agent harness includes software engineering scenarios for tests, review, firmware contracts, ROS launch/config, voice confirmation, and stale perception data.
- Hardware-facing tools produce timestamped logs or reports.

## Deferred Work

- Fully autonomous LLM-driven navigation or manipulation.
- Direct VLA/VLM policy execution on hardware without deterministic safety envelopes.
- Automatic deployment/update flow to the physical robot.
- Direct raw `/cmd_vel`, PWM, or unguarded joint-control tools.

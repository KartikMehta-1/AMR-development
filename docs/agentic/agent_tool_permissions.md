# Agent Tool Permissions

This document defines what AMR project agents may do, what requires confirmation, and what is blocked.

The agent layer is an operator/developer interface above ROS 2, Nav2, MoveIt2, `ros2_control`, micro-ROS, and STM32 firmware. It must not become a parallel robot control system.

## Permission Classes

### Class 0 - Documentation And Read-Only Repo Inspection

Allowed without confirmation:

- Read files in the repository.
- Search code/docs with `rg`, `find`, `git diff`, and similar read-only commands.
- Summarize architecture, logs, scripts, configs, and code.
- Inspect git status.

Examples:

- Read `docs/architecture/ros_stack_diagrams.md`.
- Inspect `ros_ws/src/amr_missions`.
- Summarize changed files.

Failure modes:

- Agent gives outdated guidance because it did not inspect current files.
- Agent misses uncommitted user changes and overwrites them later.
- Agent treats old docs as source of truth when code has moved on.

Required mitigation:

- Check current files before recommendations.
- Check `git status` before edits.
- Prefer current code and launch files over older notes when conflicts exist.

### Class 1 - Software-Only Local Commands

Allowed without hardware confirmation when they do not start robot processes:

- `colcon build` for selected packages or the workspace.
- `colcon test`.
- Python unit tests.
- Static checks and format checks.
- Firmware compile checks when the command is documented and does not flash hardware.

Examples:

- Build `amr_voice`, `amr_missions`, `amr_safety`.
- Run parser tests.
- Run launch/config smoke checks that do not connect to hardware.

Failure modes:

- Build command accidentally uses stale environment.
- Test depends on unavailable ROS setup.
- Long-running command hangs.
- Generated build/log files dirty the repo.

Required mitigation:

- Report exact commands run.
- Distinguish pass, fail, skipped, and not runnable.
- Avoid claiming hardware validation from software-only checks.
- Do not delete user files or generated artifacts without explicit request.

### Class 2 - Read-Only Robot/ROS Diagnostics

Allowed only when the robot/ROS runtime is expected to be running, but these tools must not command motion:

- List ROS nodes, topics, services, actions.
- Read topic samples.
- Inspect TF availability.
- Read mission state, safety state, fault masks, localization status, Nav2 status.
- Read logs and baseline reports.

Examples:

- `ros2 topic echo /amr/safety_state --once`
- Inspect `/amr_missions/status`.
- Check whether `map -> odom` exists.

Failure modes:

- Topic echo blocks or reads stale data.
- Agent misinterprets missing data as healthy data.
- Runtime command changes timing on a weak system.
- Agent samples too little data and misses intermittent faults.

Required mitigation:

- Use bounded timeouts where possible.
- Report data age and missing topics explicitly.
- Never infer motion readiness from one signal alone.
- Treat missing safety/localization state as not ready.

### Class 3 - Repo Edits And Code Generation

Allowed when the user asks for implementation or documentation changes:

- Edit docs, scripts, ROS packages, tests, and firmware source.
- Add tests and harness scenarios.
- Refactor shared client logic.
- Update package metadata when needed.

Examples:

- Add a parser test.
- Add a safety-supervisor unit test.
- Update `docs/agentic/agentic_robotics_roadmap.md`.
- Modify STM firmware only within the requested scope.

Failure modes:

- Agent changes unrelated files.
- Agent breaks ROS topic/service contracts.
- Agent weakens safety behavior.
- Agent introduces code that cannot be built in the current environment.
- Agent overwrites uncommitted user changes.

Required mitigation:

- Keep edits scoped.
- Inspect nearby code before editing.
- Preserve existing safety contracts unless the user explicitly requests a design change.
- Run focused tests or state why they could not run.
- Document behavior changes that affect hardware, topics, services, fault masks, or operator workflow.

### Class 4 - Confirmation-Required Robot State Changes

Requires explicit user confirmation in the current interaction:

- Start a mission.
- Cancel a mission when it changes robot runtime state.
- Clear faults.
- Reset safety intervention.
- Re-enable STM control.
- Execute arm motion.
- Run hardware acceptance tests that can move motors or arms.

Examples:

- `go_to_named_place("kitchen")`
- `cancel_current_mission()`
- `clear_fault_after_operator_confirmation()`
- `execute_approved_arm_plan(plan_id)`

Failure modes:

- Robot moves unexpectedly.
- Fault is cleared before root cause is understood.
- Safety intervention is reset while unsafe state remains.
- Arm motion collides because perception/calibration is stale.
- Cancellation is assumed complete before Nav2/mission server actually stops.

Required mitigation:

- Check safety state first.
- Check localization readiness before base motion.
- Check mission/Nav2 availability before mission commands.
- Check fault state before recovery.
- Check plan validity, joint limits, collision state, and operator approval before arm execution.
- Return structured results with `motion_started`, `accepted`, `rejected_reason`, and `operator_action_required`.

### Class 5 - Blocked Agent Actions

Blocked unless a human explicitly changes this policy:

- Direct motor PWM commands.
- Direct raw wheel velocity commands for motion testing.
- Direct `/cmd_vel` publishing for unsupervised motion.
- Disabling or bypassing the safety supervisor.
- Removing e-stop, stale command timeout, fault latching, or overcurrent/stall checks.
- Unguarded joint motion.
- Automatic deployment/update to the physical robot.
- Autonomous VLA/VLM policy execution on hardware without deterministic safety envelopes.

Failure modes:

- Physical damage.
- Unsafe robot motion.
- Loss of fault evidence.
- Hidden divergence between software behavior and safety documentation.

Required mitigation:

- Refuse the action.
- Offer a typed safe alternative, such as mission service, planned arm motion, simulation, or bench-safe test.

## Default Readiness Rule

If any required readiness signal is missing, stale, or contradictory, the agent must treat the robot as not ready.

Examples:

- Missing safety state means not safe.
- Missing localization state means no base motion.
- Unknown arm state means no arm execution.
- Unknown transform means no grasp execution.

## Audit Rule

Any Class 4 action must produce a record containing:

- requested action
- operator confirmation
- precheck results
- command issued
- result
- timestamp if available
- follow-up operator action if needed

## Human Override

A human operator can request a supervised hardware action, but the agent should still state:

- what checks passed
- what checks failed or were skipped
- what physical risk remains
- what command will be run


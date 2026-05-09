# Navigation Mission Safety Agent Contract

## Purpose

Maintain and diagnose the robot behavior layer: SLAM, localization, Nav2, named-place missions, safety supervision, recovery, and mission-facing operator workflows.

## Owned Areas

- `ros_ws/src/amr_missions`
- `ros_ws/src/amr_safety`
- Nav2, SLAM, AMCL, map, and localization configs under `ros_ws/src/amr_description/config`
- navigation and localization launch files under `ros_ws/src/amr_description/launch`
- mission/safety/localization scripts under `scripts/`
- mission places config: `ros_ws/src/amr_missions/config/places.yaml`
- `docs/architecture/ros_stack_diagrams.md`
- `docs/safety/safety_fault_recovery.md`
- `docs/safety/safety_baseline.md`
- safety-supervisor step docs

## Allowed Commands

- Read and edit owned files when implementing requested changes.
- `colcon build` for affected packages when environment is available.
- `colcon test` for affected packages when tests exist.
- Read-only ROS diagnostics for safety state, mission state, localization, Nav2, TF, odom, and scan when runtime diagnosis is requested.
- Read-only repo commands such as `rg`, `find`, `sed`, `git diff`.

## Blocked Commands

- Starting a mission without explicit confirmation.
- Clearing faults or resetting safety intervention without explicit confirmation.
- Re-enabling STM automatically.
- Direct `/cmd_vel` or wheel-command publishing for motion.
- Running SLAM and AMCL together in a way that conflicts over `map -> odom`.
- Bypassing `mission_server` or `safety_supervisor`.

## Required Checks

For mission changes:

- Validate named places and request inputs.
- Preserve bounded timeouts.
- Preserve cancellation behavior.
- Add or update tests for deterministic mission validation logic.

For safety changes:

- Preserve unsafe-state denial.
- Preserve guarded zero velocity / disable flow.
- Preserve manual re-enable unless explicitly redesigned.
- Update docs for changed fault or recovery behavior.

For navigation/localization changes:

- Preserve TF ownership rules.
- Check interaction between SLAM, AMCL, map server, odom, and robot state publisher.
- Preserve topic remaps between Nav2 and the diff-drive control path.
- Treat missing or stale localization as not ready for motion.

For recovery:

- Never clear faults just to make a workflow proceed.
- Confirm current fault state and root-cause evidence before reset guidance.
- Keep audit records for Class 4 recovery actions.

## Done Criteria

- Focused build/test result is reported or blocker is documented.
- Changed mission/safety behavior is tested when software-only testing is possible.
- Operator-facing workflow changes are documented.
- No direct control path bypasses mission/safety layers.

## Common Failure Modes

- A new mission command bypasses safety or localization readiness.
- A service call hangs because timeout behavior is missing.
- A safety reset is allowed while STM faults remain active.
- AMCL/SLAM or TF ownership conflicts are introduced.
- Nav2 tuning is changed before odom/localization evidence is checked.

## Escalation Rules

- If a request requires physical robot motion, ask for explicit supervised confirmation.
- If safety behavior must change, document the design reason before editing.
- If navigation failures implicate wheel state, current, fault masks, or micro-ROS, coordinate with ROS Core Hardware Interface and STM Firmware agents.


# Manipulator MoveIt Agent Contract

## Purpose

Develop the SO-101 manipulation stack once URDF, MoveIt2, joint drivers, and calibration work become active. This contract is intentionally light for now and should be expanded as the manipulator software lands in the repo.

## Owned Areas

- Future SO-101 URDF/Xacro files.
- Future MoveIt2 config packages.
- Future arm driver, trajectory, gripper, and joint-state packages.
- Arm-related calibration docs and tool/gripper frame docs.
- Manipulation portions of launch files and hardware acceptance checks.

## Allowed Commands

- Read and edit manipulator-owned files when requested.
- Run software-only planning/config checks when available.
- Run simulation or bench-safe checks only when they do not move hardware.
- Read-only repo commands such as `rg`, `find`, `sed`, `git diff`.

## Blocked Commands

- Unguarded joint motion.
- Executing a trajectory without an approved plan.
- Bypassing MoveIt2/planning limits with direct joint commands.
- Ignoring collision geometry, joint limits, or stale arm state.
- Running hardware arm motion without explicit confirmation.

## Required Checks

- Verify joint limits, velocity limits, and named poses.
- Verify frame IDs for base, arm, wrist, tool, gripper, and camera.
- Keep planning separate from execution.
- Require approval before execution on hardware.
- Coordinate with perception/calibration when grasp targets are involved.

## Done Criteria

- Planning works in simulation or bench-safe mode when available.
- Named poses and limits are documented.
- Execution plan is separate from execution approval.
- Hardware validation plan is provided for physical arm behavior.

## Common Failure Modes

- Direct joint commands bypass planning.
- Collision model is incomplete.
- Tool or camera frame is stale or incorrect.
- Planning succeeds in sim but hardware limits differ.

## Escalation Rules

- If the arm may move, require explicit supervised confirmation.
- If object pose comes from perception, coordinate with the Perception Calibration Agent.
- If base motion and arm motion are coupled, coordinate with Navigation Mission Safety Agent.


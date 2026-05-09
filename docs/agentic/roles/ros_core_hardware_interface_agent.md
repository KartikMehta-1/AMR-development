# ROS Core Hardware Interface Agent Contract

## Purpose

Maintain the ROS 2 workspace structure and the ROS-to-hardware interface layer that connects Nav2/mission logic to `ros2_control`, `amr_hardware`, and micro-ROS topic contracts.

## Owned Areas

- `ros_ws/src/amr_hardware`
- `ros_ws/src/amr_clients` for shared ROS client helpers and safe wrapper patterns
- `ros_ws/src/amr_description` when changes affect URDF, ros2_control, controller configs, or hardware bring-up
- `ros_ws/src/amr_missions_msgs` when message/service contracts are shared across packages
- ROS package metadata, build files, launch conventions, and shared interface patterns
- micro-ROS topic contracts between ROS 2 and STM firmware
- `docs/architecture/ros_stack_diagrams.md`
- hardware-interface portions of `docs/architecture/STM_architecture.md` and fault/topic docs

## Allowed Commands

- Read and edit owned ROS files when implementing requested changes.
- `colcon build` for affected packages when environment is available.
- `colcon test` for affected packages when tests exist.
- Read-only repo commands such as `rg`, `find`, `sed`, `git diff`.
- Read-only ROS graph/topic/service inspection when runtime is expected to be active and diagnosis is requested.

## Blocked Commands

- Direct wheel command publishing for motion.
- Direct `/cmd_vel` publishing for unsupervised motion.
- Starting missions or motor tests without explicit confirmation.
- Changing STM topic names, units, or semantics without synchronized firmware/docs updates.
- Bypassing `ros2_control`, `amr_hardware`, or safety supervision with ad hoc command paths.
- Destructive git commands.

## Required Checks

For ROS interface changes:

- Confirm package manifests and build files stay consistent.
- Preserve message/service/action contracts or document versioned changes.
- Check topic names, QoS assumptions, units, and frame IDs.
- Verify downstream users: mission, safety, Nav2, scripts, voice, and STM firmware.

For hardware interface changes:

- Preserve left/right wheel semantics.
- Preserve wheel command/state units.
- Preserve fault/safety topic compatibility.
- Coordinate with STM firmware changes when topic contracts change.

For URDF/controller changes:

- Preserve TF ownership rules.
- Keep `ros2_control` controller names and remaps aligned with launch/config files.
- Avoid changing frames or joints without updating docs and affected configs.

## Done Criteria

- Focused `colcon build` result is reported or blocker is documented.
- Interface changes are documented.
- Affected packages/scripts are identified.
- No parallel hardware command path is introduced.

## Common Failure Modes

- Topic names drift between firmware, `amr_hardware`, configs, and scripts.
- Left/right wheel semantics get swapped.
- Controller remaps break Nav2 command flow.
- URDF frame changes break localization, Nav2, or future manipulator frames.
- Shared message/service changes are not reflected in clients.

## Escalation Rules

- If a change touches both ROS and STM firmware contracts, coordinate with the STM Firmware Agent.
- If a change can cause physical motion, require explicit supervised confirmation.
- If source-of-truth docs and code disagree, document the mismatch before continuing.

---
name: amr-ros-core-hardware-dev
description: "Use when implementing or diagnosing AMR ROS 2 core and hardware-interface changes: amr_hardware, ros2_control, URDF/Xacro, controller configs, launch files, shared messages/services, STM topic contracts, TF, and package metadata."
---

# AMR ROS Core Hardware Dev

Use this skill for ROS core, hardware bridge, `ros2_control`, URDF, controller config, launch, shared interface, or STM topic-contract work.

## Source Of Truth

Read first:

- `docs/agentic/roles/ros_core_hardware_interface_agent.md`
- `docs/architecture/ros_stack_diagrams.md`

Read when relevant:

- `ros_ws/src/amr_hardware`
- `ros_ws/src/amr_description`
- `ros_ws/src/amr_missions_msgs`
- `docs/architecture/STM_architecture.md`
- `docs/agentic/agent_tool_permissions.md`

## Workflow

1. Identify affected packages and downstream consumers.
2. Preserve `/amr_stm/*` topic names, wheel units, left/right semantics, controller names, remaps, and TF ownership unless intentionally changing them.
3. Keep URDF/Xacro, `ros2_control.yaml`, launch files, and docs synchronized.
4. Coordinate with STM Firmware Agent when firmware topic units or semantics change.
5. Do not start hardware launch files, motor drivers, or Nav2 missions without explicit supervised confirmation.
6. Run focused `colcon` build/test only when the ROS environment is available.

## Safe Checks

```bash
colcon build --packages-select amr_hardware amr_description amr_missions_msgs
colcon test --packages-select amr_hardware amr_description amr_missions_msgs
```

Use source inspection when ROS environment is unavailable.

## Output Format

```text
ROS Interface Scope
- ...

Contracts Affected
- Topics:
- Frames:
- Controllers:
- Packages:

Checks Run
- ...

Hardware Runtime Not Run
- ...

Next Step
- ...
```

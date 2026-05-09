---
name: amr-manipulator-bringup
description: "Use when developing or reviewing SO-101 manipulator URDF, MoveIt2, arm drivers, gripper, joint limits, named poses, planning scene, calibration frames, or guarded arm execution. Requires plan-before-execute and explicit approval before hardware motion."
---

# AMR Manipulator Bringup

Use this skill for SO-101 arm integration, MoveIt, arm control, gripper behavior, named poses, planning scene, or bench bring-up.

## Source Of Truth

Read first:

- `docs/agentic/roles/manipulator_moveit_agent.md`
- `docs/agentic/agent_tool_permissions.md`

Read when relevant:

- `docs/architecture/ros_stack_diagrams.md`
- future SO-101 URDF/Xacro packages
- future MoveIt config packages
- future arm calibration docs

## Workflow

1. Determine whether the task is model/config, planning-only, bench hardware, or integrated base+arm behavior.
2. Preserve joint limits, velocity/acceleration limits, collision geometry, and tool/gripper frames.
3. Require planning before execution.
4. Require explicit operator approval before hardware arm motion.
5. Treat perception grasp outputs as proposals, not commands.
6. Coordinate with Perception / Calibration when object poses or camera frames are involved.
7. Coordinate with Navigation / Mission / Safety when base pose or mission state affects arm execution.

## Blocked Unless Explicitly Requested

- Live arm execution
- Gripper actuation
- Homing against real hardware
- Writing directly to servo buses or joint command topics
- Executing perception-generated grasps without planning and approval

## Output Format

```text
Manipulator Scope
- ...

Planning/Safety Checks
- ...

Commands Run
- ...

Hardware Motion Not Run
- ...

Next Safe Step
- ...
```

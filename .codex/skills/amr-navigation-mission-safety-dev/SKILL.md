---
name: amr-navigation-mission-safety-dev
description: "Use when implementing AMR navigation, mission, safety, localization, recovery, Nav2, SLAM, AMCL, named-place, or safety-supervisor changes. Preserves guardrails and avoids live motion unless explicitly supervised."
---

# AMR Navigation Mission Safety Dev

Use this skill for code or config changes in navigation, localization, missions, safety supervisor, recovery behavior, or related scripts.

## Source Of Truth

Read first:

- `docs/agentic/roles/navigation_mission_safety_agent.md`
- `docs/architecture/ros_stack_diagrams.md`
- `docs/safety/safety_fault_recovery.md`

Read when relevant:

- `ros_ws/src/amr_missions`
- `ros_ws/src/amr_safety`
- `ros_ws/src/amr_description/config/nav2_params_amr.yaml`
- `ros_ws/src/amr_description/launch`
- `scripts`

## Workflow

1. Classify the work: Nav2/SLAM/AMCL config, mission runtime, safety supervisor, recovery script, named place, or localization helper.
2. Preserve safety denial when STM fault mask or comm fault is active.
3. Preserve mission/Nav2 cancel and zero-velocity behavior during safety intervention.
4. Keep STM re-enable manual unless explicitly redesigned.
5. Coordinate with ROS Core / Hardware Interface when topics, launch, TF, or controller wiring changes.
6. Coordinate with STM Firmware Agent when fault masks, `/amr_stm/*`, current, or wheel-state behavior changes.

## Safe Checks

```bash
python3 -m compileall ros_ws/src/amr_missions ros_ws/src/amr_safety scripts
colcon build --packages-select amr_missions amr_missions_msgs amr_safety
colcon test --packages-select amr_missions amr_missions_msgs amr_safety
```

Do not run live missions, recovery, or hardware scripts without explicit supervised confirmation.

## Output Format

```text
Subsystem Scope
- ...

Safety Contracts
- ...

Checks Run
- ...

Hardware/Motion Not Run
- ...

Next Step
- ...
```

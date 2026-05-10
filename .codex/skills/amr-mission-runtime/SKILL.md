---
name: amr-mission-runtime
description: "Use when developing or diagnosing AMR mission runtime behavior, named places, mission CLI/server, patrol/go_to/cancel/status commands, mission messages, or mission integration with Nav2 and safety. Keeps mission logic above Nav2 and safety checks."
---

# AMR Mission Runtime

Use this skill for `amr_missions`, named places, mission services/topics, mission CLI behavior, patrol sequencing, or mission status diagnosis.

## Source Of Truth

Read first:

- `docs/agentic/roles/navigation_mission_safety_agent.md`
- `docs/architecture/ros_stack_diagrams.md`
- `docs/agentic/amr_bringup_runbooks.md`

Read when relevant:

- `ros_ws/src/amr_missions`
- `ros_ws/src/amr_missions_msgs`
- `ros_ws/src/amr_missions/config/places.yaml`
- `docs/agentic/agent_tool_permissions.md`

## Workflow

1. Classify the change: config-only, CLI/client, server runtime, message/service contract, or Nav2/safety integration.
2. Preserve typed mission requests and status outputs.
3. Keep named-place validation deterministic.
4. Keep mission cancellation and timeout behavior explicit.
5. Do not bypass safety supervisor or publish direct raw motion commands.
6. For code changes, add or update software-only tests where possible.
7. Coordinate with Voice / Operator Interface when voice commands call mission behavior.

## Mission Bringup Boundary

Mission runtime may be launched only after hardware and navigation/localization readiness are established for the requested supervised context.

Required readiness checks before any mission command:

- `mission_server` starts after `amr_missions_msgs`, `amr_clients`, and `amr_missions` are built and sourced.
- `/amr_missions/list_places` returns named places.
- `/amr_missions/state` returns `idle`.
- `/amr/safety_supervisor/status` is receivable and healthy.
- Nav2 lifecycle and localization checks are green.

This skill may inspect status and list places without motion. It must not run `go_to`, `patrol`, recovery experiments, direct Nav2 goals, or raw velocity commands unless the user explicitly approves supervised motion.

## Safe Checks

Use when available and relevant:

```bash
python3 -m compileall ros_ws/src/amr_missions
colcon build --packages-select amr_missions amr_missions_msgs
colcon test --packages-select amr_missions amr_missions_msgs
```

Do not run a live `go_to`, `patrol`, or `cancel` command unless explicitly requested in a supervised runtime context.

## Output Format

```text
Mission Scope
- ...

Behavior/Contract Impact
- ...

Checks Run
- ...

Safety/Motion Not Run
- ...

Next Step
- ...
```

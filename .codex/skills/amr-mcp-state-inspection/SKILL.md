---
name: amr-mcp-state-inspection
description: "Use when developing, launching, or validating AMR MCP servers, including read-only state inspection and guarded mission-control tools for named-place missions. Must not launch hardware implicitly, publish raw motion, clear faults, reset safety, or re-enable STM."
---

# AMR MCP State Inspection

Use this skill for AMR MCP tool definitions, stdio JSON-RPC checks, read-only robot-state inspection, and guarded named-place mission control.

## Source Of Truth

Read first:

- `docs/agentic/amr_bringup_runbooks.md`
- `mcp_servers/amr_state_inspection/README.md`
- `mcp_servers/amr_mission_control/README.md`

Read when relevant:

- `mcp_servers/amr_state_inspection/server.py`
- `mcp_servers/amr_mission_control/server.py`
- `ros_ws/src/amr_clients`
- `docs/agentic/agent_tool_permissions.md`

## Workflow

1. Classify the task as source-only, MCP process validation, runtime read-only inspection, or supervised mission control.
2. MCP attaches to an already-running Foxy graph; it must not implicitly launch Jetson hardware, Nav2, missions, or recovery.
3. Inside `amr_devpc`, use:

```bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/foxy/setup.bash
source /workspaces/AMR-development/ros_ws/install/setup.bash
```

4. Validate each server over stdio, not only by importing Python classes:
   - `initialize`
   - `tools/list`
   - representative `tools/call`
5. State-inspection expected tools:
   - `get_robot_health`
   - `get_safety_state`
   - `get_localization_state`
   - `get_mission_state`
   - `list_named_places`
   - `get_stm_diagnostics`
   - `get_navigation_state`
6. Mission-control expected tools:
   - `list_named_places`
   - `get_mission_state`
   - `check_go_to_readiness`
   - `go_to_named_place`
   - `cancel_mission`

## Readiness Criteria

- `list_named_places` returns known places.
- `get_mission_state` returns `idle` for bringup readiness.
- `get_stm_diagnostics` reports zero STM and communication fault masks.
- `get_safety_state` reports healthy supervisor status.
- `get_navigation_state` reports active Nav2 lifecycle nodes.
- `get_localization_state` reports fresh scan, fresh AMCL pose, and `map -> odom`.
- `get_robot_health` separates blockers and warnings instead of hiding partial failures.

## Mission Control Criteria

- `check_go_to_readiness` must validate place existence, mission idle state, safety health, localization freshness, and Nav2 lifecycle state.
- `go_to_named_place` must call `/amr_missions/go_to` through `MissionClient`; it must not publish `/cmd_vel` or call Nav2 actions directly.
- Live `go_to_named_place` requires `operator_confirmed_supervised=true`; dry-run must be available and must not start motion.
- `cancel_mission` may request mission-server cancellation only. It must not clear faults, reset intervention, or re-enable STM.

## Blocked Actions

MCP work must not:

- Start hardware or Nav2 containers implicitly.
- Publish `/cmd_vel`.
- Start direct Nav2 goals or bypass `mission_server`.
- Clear faults, reset safety intervention, or re-enable STM.
- Treat stale localization as motion-ready.

## Safe Checks

```bash
python3 -m py_compile mcp_servers/amr_state_inspection/server.py
python3 -m py_compile mcp_servers/amr_mission_control/server.py
```

Run live MCP checks only when the user expects the AMR runtime to be active. For mission-control validation, prefer `check_go_to_readiness` and `go_to_named_place` with `dry_run=true` unless the user explicitly approves supervised motion.

## Output Format

```text
MCP Scope
- ...

Runtime Attachment
- Foxy container:
- Graph already running:

Tool Results
- ...

Blocked/Motion Actions Not Run
- ...

Next Step
- ...
```

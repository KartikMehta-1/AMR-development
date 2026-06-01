# AMR MCP Validation - 2026-05-30

This checkpoint validates the AMR MCP surface before wiring it into a higher-level
OpenClaw main-agent setup. No live mission, teleop, raw `/cmd_vel`, fault clear,
safety reset, or STM re-enable command was sent during this validation.

## Runtime Context

- Host: NUC at `/home/ubuntu/agent/repos/AMR-development`.
- Active launch: one-command NUC bringup via `scripts/open_amr_devpc_navigation.sh my_new_map`.
- Containers:
  - NUC `amr_devpc`: running.
  - Jetson `amr_foxy`: running.
- ROS graph:
  - Map ready from `my_new_map`.
  - AMCL localized near known `home` pose.
  - Mission server running and idle.
  - Safety supervisor healthy in monitor-only mode.
  - STM fault mask `0`, comm status `stm_link_ok`.

## Source-Only Checks

Passed:

```bash
python3 -m py_compile mcp_servers/*/server.py mcp_servers/*/smoke_test.py
python3 agent_harness/software/validate_harness.py
python3 agent_harness/software/run_static_contract_checks.py
```

## MCP Smoke Tests

Passed on host:

- `mcp_servers/amr_conversation/smoke_test.py`
- `mcp_servers/amr_perception_inspection/smoke_test.py`
- `mcp_servers/amr_robot_launch/smoke_test.py`
- `mcp_servers/amr_speaker/smoke_test.py`
- `mcp_servers/amr_voice_interface/smoke_test.py`

Passed inside `amr_devpc` with Foxy workspace sourced:

- `mcp_servers/amr_state_inspection/smoke_test.py`
- `mcp_servers/amr_mission_control/smoke_test.py`

`amr_state_inspection` and `amr_mission_control` are ROS-attached MCPs and should
be validated inside the sourced Foxy workspace when a live graph is expected.

## Live Read-Only / Dry-Run Validation

`amr_state_inspection` live checks returned `ok=true` with no blockers:

- `get_robot_health`
- `get_safety_state`
- `get_localization_state`
- `get_navigation_state`
- `get_mission_state`
- `list_named_places`
- `get_stm_diagnostics`

`amr_mission_control` live dry-run checks returned `ok=true` with no blockers:

- `check_go_to_readiness(place="hall")`
- `go_to_named_place(place="hall", dry_run=true, operator_confirmed_supervised=false)`

`amr_robot_launch` host-side checks returned `ok=true` with no blockers:

- `get_launch_status`
- `preflight_launch(map="my_new_map")`
- `launch_navigation_stack(map="my_new_map", dry_run=true, operator_confirmed_supervised=false)`

## OpenClaw Integration Status

MCPs are ready to be registered behind an OpenClaw main agent with these boundaries:

- Prefer `amr_state_inspection` for read-only status.
- Use `amr_robot_launch` for guarded bringup.
- Use `amr_mission_control` for named-place missions only after readiness and
  explicit supervised operator confirmation.
- Use `amr_voice_interface`, `amr_conversation`, and `amr_speaker` as operator
  interface layers; they must not replace mission-control readiness gates.

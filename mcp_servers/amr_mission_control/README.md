# AMR Mission Control MCP Server

Guarded MCP server for named-place mission execution through `mission_server`.

This server can request a mission such as `go_to_named_place(place="kitchen")`, but only through the typed `/amr_missions/go_to` service. It does not publish `/cmd_vel`, call Nav2 actions directly, clear faults, reset safety intervention, or re-enable STM.

## Tools

- `list_named_places`
- `get_mission_state`
- `check_go_to_readiness`
- `go_to_named_place`
- `cancel_mission`

## Runtime

Run inside the Foxy dev-PC container after the workspace has been built and sourced:

```bash
cd /workspaces/AMR-development
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/foxy/setup.bash
source ros_ws/install/setup.bash
python3 mcp_servers/amr_mission_control/server.py
```

## Execution Rules

Before `go_to_named_place` starts a live mission, it checks:

- The requested place exists.
- Mission state is idle or otherwise not actively running.
- Safety supervisor and STM diagnostics are healthy.
- Localization has fresh `/amcl_pose`, fresh `/scan`, and `map -> odom`.
- Nav2 lifecycle nodes are active.
- `operator_confirmed_supervised` is true.

Use `check_go_to_readiness` or `go_to_named_place` with `dry_run=true` for non-motion validation.

## Environment

- `AMR_MISSION_MCP_TIMEOUT_SEC`: default ROS graph/service timeout, default `5.0`.
- `AMR_MISSION_MCP_GOAL_TIMEOUT_SEC`: default mission goal timeout, default `180.0`.
- `AMR_MISSION_MCP_MAX_POSE_AGE_SEC`: default `3.0`.
- `AMR_MISSION_MCP_MAX_SCAN_AGE_SEC`: default `1.0`.

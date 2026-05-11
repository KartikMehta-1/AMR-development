# AMR State Inspection MCP Server

Read-only MCP server for AMR state inspection.

This server is intentionally limited to observation tools. It must not expose mission start, motion, fault clear, STM enable, reset, or arm-control tools.

## Tools

- `get_robot_health`
- `get_safety_state`
- `get_localization_state`
- `get_mission_state`
- `list_named_places`
- `get_stm_diagnostics`
- `get_navigation_state`
- `get_last_known_place`

## Runtime

Run inside the current Foxy Docker environment after the ROS workspace has been built and sourced:

```bash
cd /workspaces/AMR-development/ros_ws
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 ../mcp_servers/amr_state_inspection/server.py
```

The default ROS graph timeout is three seconds. Override it only when needed:

```bash
AMR_MCP_TIMEOUT_SEC=5 python3 ../mcp_servers/amr_state_inspection/server.py
```

For software-only validation without a live ROS graph, the server should still start and return unavailable responses for graph-dependent tools.

Do not run this server as a substitute for hardware acceptance. It only reports state exposed by existing ROS topics and services.

`get_last_known_place` reads the persisted place record written by `mission_server` after a successful named-place mission. By default this is `ros_ws/log/amr_last_place.json` inside the mounted workspace, or `AMR_MISSION_LAST_PLACE_PATH` when set. Treat this as an initial-pose hint for the next launch, not as proof of current localization.

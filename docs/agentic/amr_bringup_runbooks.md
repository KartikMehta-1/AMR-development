# AMR Agent Bringup Runbooks

These runbooks capture the current Jetson Nano + ROS 2 Foxy AMR bringup contract. They are written for domain agents. They do not override the hardware permission rules in `agent_tool_permissions.md`.

## Shared Runtime Rules

- Current robot runtime is Foxy in Docker, not host ROS and not Humble.
- Inside AMR containers, always use:
  - `unset CYCLONEDDS_URI`
  - `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
  - `source /opt/ros/foxy/setup.bash`
  - `source /workspaces/AMR-development/ros_ws/install/setup.bash` on the dev PC container
  - `source /workspaces/ros_ws/install/setup.bash` on the Jetson container
- Do not treat a process as healthy just because it exists. Verify the ROS topic, service, TF, lifecycle, or action contract the next layer needs.
- Before relaunching Nav2/RViz, clean up stale `rviz2`, `bringup_nav2`, `map_server`, `amcl`, Nav2 lifecycle nodes, and `amr_static_map_publisher.py` processes in the dev-PC container.
- Do not send `/cmd_vel`, start a mission, clear faults, reset safety intervention, or re-enable STM unless the user explicitly approves that action for a supervised run.

## Hardware Agent: Limited Jetson Bringup

Goal: prove the robot hardware graph exists without starting navigation or missions.

Owned launch scope:

- Jetson `amr_foxy` Foxy container.
- `hardware.launch.py` with STM micro-ROS agent, `amr_hardware`, robot state publisher, controllers, LiDAR, and optional link watchdog.

Required verification:

- `ros2 control list_controllers` shows `joint_state_broadcaster` and `diff_drive_controller` active.
- `/amr_stm/wheel_state` is receivable with best-effort QoS.
- `/amr_stm/fault_mask`, `/amr_stm/comm_status`, and `/amr_stm/comm_fault_mask` are receivable.
- `/scan` is present when LiDAR is requested.
- `/odom`, `/tf`, and `/tf_static` are present.
- No STM fault mask or communication fault mask is nonzero unless explicitly being diagnosed.

Blocked in limited bringup:

- Nav2, AMCL, SLAM, mission server, teleop, voice, and MCP motion tools.

## Navigation Agent: Map, RViz, AMCL, Nav2

Goal: prove map display and localization readiness before missions are enabled.

Owned launch scope:

- Dev-PC `amr_devpc` Foxy container.
- `amr_scan_sanitizer.py`, `bringup_nav2.launch.py`, RViz, AMCL, and Nav2 lifecycle nodes.

Required map checks:

- Confirm selected map path and metadata. Current known maps:
  - `my_new_map.yaml`: `259 x 160`, resolution `0.05`, origin `[-3.72, -1.4, 0]`.
  - `my_hall_save.yaml`: `216 x 299`, resolution `0.05`, origin `[-7.16, -7.53, 0]`.
- Verify `/map` with a late subscriber, not only with map-server logs:

```bash
python3 /workspaces/AMR-development/scripts/amr_wait_for_map.py --timeout 12
```

- If `map_server` logs a successful load but `/map` is not receivable, start the guarded fallback publisher:

```bash
python3 /workspaces/AMR-development/scripts/amr_static_map_publisher.py <map.yaml>
```

The fallback publisher only publishes `nav_msgs/OccupancyGrid` on `/map`. It does not command motion.

Required localization checks:

- RViz has a Map display using `/map`, reliable + transient-local QoS, fixed frame `map`.
- Operator gives `2D Pose Estimate` in RViz after the correct map is visible.
- `amr_wait_for_localization.py` reports fresh `/amcl_pose` and `map -> odom`.
- Nav2 lifecycle nodes are active enough for missions: controller, planner, recoveries, BT navigator, AMCL, and map server where lifecycle discovery is available.

## Mission And Safety Agent

Goal: expose mission APIs only after navigation is initialized.

Owned launch scope:

- `mission_server`.
- `safety_supervisor` in monitor-only mode during normal bringup.
- Mission and safety monitor panes.

Required verification:

- Build includes `amr_missions_msgs`, `amr_clients`, `amr_missions`, and `amr_safety` when present.
- Restart mission server after rebuilding mission packages.
- `/amr_missions/list_places` returns named places.
- `/amr_missions/state` returns `idle` before missions are enabled.
- `/amr/safety_supervisor/status` is receivable.
- Safety supervisor remains `enforce=false` unless the test is explicitly about enforcement.

Blocked:

- No `go_to`, `patrol`, cancel/recovery experiment, safety reset, or STM re-enable without explicit supervised confirmation.

## MCP Agent

Goal: expose explicit MCP surfaces for AMR graph inspection and supervised mission control.

Owned launch scope:

- Read-only MCP server under `mcp_servers/amr_state_inspection`.
- Guarded mission-control MCP server under `mcp_servers/amr_mission_control`.
- Guarded host-side launch MCP server under `mcp_servers/amr_robot_launch`.
- Voice intent MCP server under `mcp_servers/amr_voice_interface`.
- Conversation turn-planning MCP server under `mcp_servers/amr_conversation`.
- Spoken-feedback MCP server under `mcp_servers/amr_speaker`.
- State-inspection and mission-control MCP servers must attach to an already-running Foxy graph. They should not implicitly launch hardware or Nav2.
- Robot-launch MCP may invoke the standard launcher only through an explicit supervised operator confirmation flag; dry-run/preflight must remain available and non-hardware-facing.
- Voice, conversation, and speaker MCPs may plan responses, recommend safe tool calls, and publish spoken feedback, but they must not execute motion, clear faults, or replace mission-control readiness gates.

Required state-inspection verification:

- Server starts inside `amr_devpc` after the workspace is built and sourced.
- Tools list includes robot health, safety, localization, mission, named places, STM diagnostics, navigation state, and last known place.
- Health output separates blockers:
  - `/map` missing or not receivable.
  - `/amcl_pose` missing or stale.
  - `map -> odom` missing.
  - mission services unavailable.
  - STM fault or communication fault.
  - safety supervisor status missing.
- `get_last_known_place` may be used as an operator-facing initial-pose hint after launch, but it must not replace AMCL readiness checks.

Required mission-control verification:

- Server starts inside `amr_devpc` after the workspace is built and sourced.
- Tools list includes mission state, named places, go-to readiness, guarded go-to, and cancel.
- `check_go_to_readiness` must pass before any live mission is requested.
- `go_to_named_place` must call `/amr_missions/go_to`, not raw `/cmd_vel` or Nav2 actions.
- `go_to_named_place` must require explicit supervised operator confirmation for live execution.
- Dry-run calls must validate all readiness checks without starting motion.

Mission-control blockers:

- Unknown place.
- Mission is not idle.
- Safety supervisor unhealthy, intervention active, STM fault, or communication fault.
- `/amcl_pose`, `/scan`, or `map -> odom` missing/stale.
- Nav2 lifecycle nodes unavailable or not active.
- Missing supervised operator confirmation for live execution.

Required robot-launch verification:

- Server starts on the dev PC host.
- Tools list includes launch status, launch preflight, and guarded navigation-stack launch.
- `preflight_launch` validates map existence, host commands, launch script availability, and Jetson SSH reachability without starting hardware.
- `launch_navigation_stack` defaults to dry-run and must require `operator_confirmed_supervised=true` before live hardware-facing launch.
- Live launch must use the standard `scripts/open_amr_devpc_navigation.sh` path with non-attaching tmux mode so the MCP call returns after creating the operator session.

Robot-launch blockers:

- Missing map file.
- Missing `docker`, `tmux`, `ssh`, or `xhost`.
- Jetson SSH unavailable.
- Launch script missing or not executable.
- Missing supervised operator confirmation for live launch.

## Test Runner Agent

Goal: validate changes without causing motion.

Software checks for bringup changes:

- `bash -n` for touched shell scripts.
- Python syntax/import checks for touched helper scripts in the Foxy container.
- Focused `colcon build --merge-install --packages-select ...` for touched ROS packages.
- Static map probe:

```bash
python3 scripts/amr_static_map_publisher.py --help
python3 scripts/amr_wait_for_map.py --help
```

Runtime read-only checks are allowed only when the user expects the AMR runtime to be active:

- `/map` late-subscriber check.
- `/amcl_pose` freshness.
- `tf2_echo map odom`.
- mission state/list services.
- STM diagnostics topics.

## Jetson Power-On Autostart Consideration

Autostart can be useful, but should be staged:

1. Safe default: Jetson boots Docker and starts a hardware-only container with STM/LiDAR/controllers, no Nav2, no teleop, no mission server, and no automatic STM enable.
2. Health gate: publish or log hardware readiness, STM fault state, LiDAR state, controllers, and wheel-state freshness.
3. Operator/dev-PC action: start Nav2/RViz/missions from the dev PC only after the workspace is clear and the operator is ready.
4. Future supervised mode: optional full bringup service can exist, but it must be disabled by default and require an explicit enable file or systemd target.

Do not configure automatic mission execution or automatic movement on power-up.

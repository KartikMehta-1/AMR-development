---
name: amr-nav-debug
description: "Use when diagnosing AMR navigation, localization, SLAM, AMCL, Nav2, TF, odometry, scan, map, named-place, or mission navigation failures. Prioritizes read-only inspection and does not start missions or motion without explicit supervised confirmation."
---

# AMR Navigation Debug

Use this skill when the user asks why navigation, localization, mapping, or named-place movement is failing.

## Source Of Truth

Read first:

- `docs/agentic/roles/navigation_mission_safety_agent.md`
- `docs/architecture/ros_stack_diagrams.md`

Read when relevant:

- `docs/agentic/agent_tool_permissions.md`
- `ros_ws/src/amr_description/config/nav2_params_amr.yaml`
- `ros_ws/src/amr_description/launch/`
- `ros_ws/src/amr_missions/config/places.yaml`

## Workflow

1. Identify whether this is source-only, runtime read-only, simulation, or supervised hardware.
2. For source-only work, inspect launch/config/package changes and mission place definitions.
3. For runtime diagnosis, use read-only ROS checks only when the user expects ROS to be running.
4. Check the navigation chain in order:
   - `/scan`
   - `/amr_stm/wheel_state`
   - `/diff_drive_controller/odom`
   - `odom -> base_footprint`
   - `map -> odom`
   - lifecycle state for Nav2 nodes
   - mission server state
5. Do not start Nav2 goals, named-place missions, teleop, or recovery motion without explicit supervised confirmation.
6. If wheel state, STM comms, or fault masks are implicated, coordinate with STM Firmware and ROS Core / Hardware Interface contracts.

## Safe Commands

Read-only examples, only when ROS runtime is expected:

```bash
ros2 topic list
ros2 node list
ros2 lifecycle get /controller_server
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

Prefer source inspection when runtime is not active.

## Output Format

```text
Navigation Chain
- Scan:
- Wheel state:
- Odom:
- TF:
- Localization:
- Nav2:
- Mission:

Likely Root Cause
- ...

Checks Run
- ...

Hardware/Motion Not Run
- ...

Next Safe Step
- ...
```

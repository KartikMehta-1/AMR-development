# Safety Step 3: Passive Supervisor

Step 3 restores the ROS safety supervisor as a monitor-only node. It observes safety-relevant signals and publishes a decoded status, but it does not stop, disable, clear, or re-enable the robot.

## Purpose

- Validate safety thresholds without giving the supervisor authority.
- Keep the STM as the only active safety layer during this step.
- Make stale-topic, STM fault, communication fault, odom, scan, and AMCL health visible in one status topic.

## Node

```bash
ros2 run amr_safety safety_supervisor
```

The navigation script starts it automatically when `ros_ws/src/amr_safety/package.xml` exists.

Status topic:

```bash
/amr/safety_supervisor/status
```

Message type:

```text
std_msgs/String
```

The string is JSON so it can be inspected with normal `ros2 topic echo` and parsed by scripts later.

## Default Mode

```text
mode: monitor_only
action_authority: false
```

The following parameters keep Step 3 passive:

```text
monitor_only=true
enforce=false
auto_reenable_when_safe=false
```

Step 4 adds an explicit opt-in enforcement mode with `enforce:=true`; see `docs/safety/safety_supervisor_step4.md`.

## Observed Inputs

| Signal | Default Topic | Purpose |
| --- | --- | --- |
| STM fault mask | `/amr_stm/fault_mask` | Latched STM safety faults |
| STM safety state | `/amr_stm/safety_state` | STM control state and lower fault bits |
| STM comm status | `/amr_stm/comm_status` | Link watchdog status |
| STM comm fault mask | `/amr_stm/comm_fault_mask` | Link watchdog fault bits |
| Odometry | `/odom` | Controller and wheel-state path freshness |
| Lidar scan | `/scan` | Lidar freshness |
| AMCL pose | `/amcl_pose` | Localization freshness when available |

## Default Freshness Thresholds

| Signal | Threshold |
| --- | --- |
| STM fault/safety data | `0.5 s` |
| STM comm status | `1.5 s` |
| Odometry | `0.5 s` |
| Scan | `0.5 s` |
| AMCL pose | `2.0 s` |

AMCL is not required by default so the same node can run during hardware-only checks. Set `require_amcl:=true` for full navigation validation.

## Validation

Build:

```bash
cd ~/AMR-development/ros_ws
colcon build --merge-install --packages-select amr_safety
```

Run manually:

```bash
source install/setup.bash
ros2 run amr_safety safety_supervisor
```

Check status:

```bash
timeout 5 ros2 topic echo /amr/safety_supervisor/status
```

Healthy Step 3 status should show:

```text
"mode": "monitor_only"
"action_authority": false
"healthy": true
"fault_mask": 0
"comm_status": "stm_link_ok"
"comm_fault_mask": 0
```

## Exit Criteria

- Node builds and starts from the navigation script.
- Status is healthy during idle baseline.
- Status remains healthy during a short teleop or mission.
- No stop, disable, clear, or re-enable command is published by this node.

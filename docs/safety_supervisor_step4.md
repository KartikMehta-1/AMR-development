# Safety Step 4: Gated Soft Stop

Step 4 adds an active safety path, but it remains opt-in. The default supervisor mode is still monitor-only. The node gets authority only when launched with `enforce:=true`.

## Default Behavior

Default launch:

```bash
ros2 run amr_safety safety_supervisor
```

Default status remains:

```text
mode: monitor_only
action_authority: false
```

No stop, disable, clear, e-stop, or re-enable command is published in default mode.

## Enforced Behavior

Explicit enforced launch:

```bash
ros2 run amr_safety safety_supervisor --ros-args -p enforce:=true
```

When unsafe, the supervisor publishes:

| Topic | Message | Command |
| --- | --- | --- |
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` | zero velocity |
| `/amr_stm/enable` | `std_msgs/Bool` | `{data: false}` |

It does not publish:

| Topic | Reason |
| --- | --- |
| `/amr_stm/clear_fault` | faults must be captured and cleared by the operator |
| `/amr_stm/estop` | Step 4 is a soft stop/disable layer, not a latched e-stop layer |
| `/amr_stm/enable=true` | auto re-enable is intentionally deferred |

## Unsafe Conditions

The supervisor intervenes when any of these are true after the startup grace period:

- STM fault or safety-state data is stale.
- STM communication status is stale.
- Odometry is stale.
- Scan is stale.
- AMCL is stale and `require_amcl:=true`.
- `/amr_stm/fault_mask` is nonzero.
- `/amr_stm/comm_fault_mask` is nonzero.
- `/amr_stm/comm_status` is not `stm_link_ok`.

Default thresholds:

| Signal | Threshold |
| --- | --- |
| STM fault/safety data | `0.5 s` |
| STM comm status | `1.5 s` |
| Odometry | `0.5 s` |
| Scan | `0.5 s` |
| AMCL pose | `2.0 s` |
| Startup grace | `3.0 s` |

## Status Fields

`/amr/safety_supervisor/status` includes:

```text
mode
healthy
action_authority
intervention_active
intervention_count
intervention_reasons
last_intervention_reasons
fault_mask
comm_status
comm_fault_mask
stale
```

## Recovery

Step 4 does not auto-recover. After intervention:

1. Capture `/amr/safety_supervisor/status`.
2. Capture and decode STM fault state if present.
3. Fix the physical or communication cause.
4. Follow `docs/safety_fault_recovery.md`.
5. Manually re-enable STM only after the baseline probe is healthy.

## Non-Hardware Enforcement Test

This validates intervention without touching the robot command topics by remapping the action outputs:

```bash
ros2 run amr_safety safety_supervisor --ros-args \
  -p enforce:=true \
  -p startup_grace_sec:=0.0 \
  -p cmd_vel_topic:=/test_safety/cmd_vel \
  -p enable_topic:=/test_safety/enable
```

Expected:

- `/test_safety/cmd_vel` receives zero `Twist`.
- `/test_safety/enable` receives `{data: false}`.
- Status shows `mode=enforce`, `action_authority=true`, and nonempty `intervention_reasons`.

## Hardware Validation

Only run this after the monitor-only baseline is healthy:

```bash
ros2 run amr_safety safety_supervisor --ros-args -p enforce:=true
```

Initial healthy status should show:

```text
mode: enforce
action_authority: true
healthy: true
intervention_active: false
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
```

Do a short teleop or mission and confirm:

- no intervention during normal operation
- no STM fault
- no communication fault
- no odom/scan stale flags

Fault-injection tests for real intervention should be done separately and deliberately, one condition at a time.

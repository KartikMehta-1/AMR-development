# Safety Step 9: Supervisor Recovery Robustness

Step 9 improves and validates the safety supervisor recovery path after an intervention. Step 8 proved that STM and Jetson communication stayed healthy during a real motor-driver power-cut fault, but it also exposed an operator-facing problem: recovery required restarting the safety supervisor to clear latched intervention state.

This step adds an explicit reset API. It does not auto-enable STM.

## Code Change

The safety supervisor now provides:

```bash
/amr/safety_supervisor/reset_intervention
```

Service type:

```bash
std_srvs/srv/Trigger
```

Behavior:

- If the supervisor is still observing an unsafe condition, reset is rejected.
- If all monitored inputs are healthy, reset clears:
  - `intervention_active`
  - `last_intervention_reasons`
  - stale-reason dwell bookkeeping
- `intervention_count` remains cumulative for the node lifetime.
- The service never publishes `/amr_stm/enable=true`; STM re-enable remains a separate manual action.

## Packaging Finding

During Step 9, the source file and the installed Python package had diverged. The source contained the newer supervisor implementation, while the installed package still contained an older supervisor. This caused inconsistent runtime behavior and missing services.

The installed package was cleaned and rebuilt:

```bash
rm -rf ros_ws/install/lib/python3.10/site-packages/amr_safety \
       ros_ws/build/amr_safety \
       ros_ws/install/share/amr_safety

colcon build --packages-select amr_safety --symlink-install --merge-install
```

After rebuild, the installed package resolved to the current source-backed build path and exposed the reset service.

## Validation

Healthy supervisor check:

```text
mode: enforce
healthy: true
intervention_active: false
intervention_count: 0
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
safety_state: ENABLED
```

Reset while already healthy:

```text
success: true
message: intervention already clear
```

Forced unsafe condition:

```bash
ros2 topic pub -r 20 /amr_stm/comm_status std_msgs/msg/String "{data: forced_fault}"
```

Observed intervention:

```text
healthy: false
intervention_active: true
intervention_count: 1
intervention_reasons: [comm_status_forced_fault]
last_intervention_reasons: [comm_status_forced_fault]
fault_mask: 0
comm_status: forced_fault
```

Reset while unsafe:

```text
success: false
message: cannot reset intervention while unsafe: comm_status_forced_fault
```

After comm status returned to `stm_link_ok`, reset succeeded:

```text
success: true
message: intervention reset
```

Manual STM re-enable was then issued separately:

```bash
ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: true}"
```

Final supervisor status:

```text
mode: enforce
healthy: true
intervention_active: false
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
safety_state: ENABLED
```

Final baseline probe:

```text
wheel: 9.807 Hz
fault: 9.809 Hz
diag: 1.965 Hz
odom: 50.023 Hz
scan: 9.663 Hz
comm: 2.001 Hz
big_gaps: 0
fault_mask: 0
safety_state: 131072
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
baseline_result: PASS
```

Runtime graph validation:

```text
/amr_safety_supervisor
/amr/safety_supervisor/reset_intervention
/amr/safety_supervisor/status publisher count: 1
```

## Recovery Sequence

Recommended manual recovery after a real hardware fault:

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"

ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: false}"

ros2 topic pub --once /amr_stm/clear_fault std_msgs/msg/Empty "{}"

ros2 service call /amr/safety_supervisor/reset_intervention std_srvs/srv/Trigger "{}"

ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: true}"
```

Only call `reset_intervention` after the real fault source is gone. The service will reject the reset if the supervisor still sees an unsafe condition.

## Next Step

Step 10 should re-run the real motor-driver power-cut test using this reset service instead of restarting the supervisor. Pass criteria: supervisor remains in the ROS graph, status continues publishing, reset is rejected while faulted, reset succeeds after `/amr_stm/clear_fault`, and STM re-enable remains manual.

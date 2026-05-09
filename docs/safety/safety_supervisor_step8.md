# Safety Step 8: Real Motor-Driver Power-Cut Fault

Step 8 validates a real STM-reported safety source. The test cuts motor-driver power while the AMR is commanded by Nav2, then verifies that STM detects the stall, the safety supervisor intervenes, and the system can be recovered without a Jetson or STM power cycle.

## Preconditions

- AMR powered and localized.
- STM enabled.
- Safety supervisor in enforced mode.
- Motor-driver power available before starting the test.
- Clear physical area around the AMR.

Pre-test status:

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

An idle 90 second probe before the test passed:

```text
fault_mask: 0
safety_state: 131072
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
baseline_result: PASS
```

## Test Method

The mission CLI did not transition the mission server even though the mission services were present, so the test used the Nav2 action directly for this hardware-safety validation:

```bash
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.502, y: 1.523, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.557, w: 0.831}}}}"
```

The AMR started moving. Motor-driver power was then cut manually during the run.

## Observed Fault

After motor-driver power was cut, STM latched:

```text
fault_mask: 24
safety_state: 65560
comm_status: stm_link_ok
comm_fault_mask: 0
```

`fault_mask=24` decodes to:

```text
STALL_LEFT  = 8
STALL_RIGHT = 16
```

The safety supervisor observed the fault and intervened:

```text
healthy: false
intervention_active: true
intervention_count: 57
fault_mask: 24
safety_state.control_state: IDLE
```

This confirms that loss of motor-driver power during commanded motion is detected by the STM firmware stall monitor and propagated through the ROS safety supervisor path.

## Intermediate State

While motor-driver power was still off, a probe confirmed the STM and Jetson communication path stayed alive:

```text
wheel: 9.745 Hz
fault: 9.724 Hz
diag: 1.991 Hz
odom: 49.972 Hz
scan: 9.677 Hz
comm: 2.000 Hz
big_gaps: 0
fault_mask: 24
safety_state: 65560
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
baseline_result: FAIL
```

The failure was expected because `fault_mask` was nonzero. There were no boot drops, UART write failures, publish failures, or stream gaps.

Nav2 was still trying to command motion after the hardware fault, so the action was cancelled through the hidden action cancel service:

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
```

## Recovery

After motor-driver power was restored, recovery was:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: false}"

ros2 topic pub --once /amr_stm/clear_fault std_msgs/msg/Empty "{}"

ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: true}"
```

The safety supervisor process was restarted afterward because its process remained present but its ROS node/status publisher disappeared from the graph after the event.

Post-recovery probe:

```text
wheel: 9.741 Hz
fault: 9.737 Hz
diag: 1.991 Hz
odom: 50.101 Hz
scan: 9.681 Hz
comm: 2.008 Hz
big_gaps: 0
fault_mask: 0
safety_state: 131072
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
baseline_result: PASS
```

Final supervisor status:

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

## Findings

- Real motor-driver power loss during commanded motion trips STM stall faults as expected.
- STM communication remained robust during and after the event.
- Safety supervisor enforced stop/disable behavior.
- Manual recovery works after motor-driver power is restored.
- Nav2 must be cancelled explicitly after the hardware fault.
- Safety supervisor node disappearance after the event needs follow-up; the process was alive, but `/amr/safety_supervisor/status` had no publisher until supervisor restart.

## Next Step

Step 9 is documented in `docs/safety/safety_supervisor_step9.md`. It adds and validates an explicit supervisor reset service so latched intervention state can be cleared after the fault source is gone, without restarting the supervisor and without auto-enabling STM.

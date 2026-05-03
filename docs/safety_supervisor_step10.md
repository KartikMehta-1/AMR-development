# Safety Step 10: Real Fault Recovery Without Supervisor Restart

Step 10 repeats the real motor-driver power-cut fault from Step 8, but uses the Step 9 supervisor reset service for recovery instead of restarting the safety supervisor.

## Goal

Validate that a real STM-reported stall fault can be recovered while the safety supervisor remains alive:

- Safety supervisor stays in the ROS graph.
- `/amr/safety_supervisor/status` continues publishing.
- `/amr/safety_supervisor/reset_intervention` rejects reset while the real fault is still present.
- Reset succeeds after the physical fault source is removed and STM fault bits are cleared.
- STM re-enable remains a separate manual command.

## Preconditions

- AMR powered and localized.
- Motor-driver power available at start.
- STM enabled.
- Safety supervisor running with enforcement enabled.
- Mission server and Nav2 action server available.

Pre-test status:

```text
mode: enforce
healthy: true
intervention_active: false
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
safety_state: ENABLED
```

## Test Method

The AMR was at `kitchen`, so the test started a mission toward `home`:

```bash
ros2 run amr_missions mission_cli --server-timeout 5 go_to home --timeout 120
```

The AMR started moving. Motor-driver power was then cut manually during motion.

## Faulted State

After motor-driver power was cut, STM reported:

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

The reset service correctly rejected recovery while the fault was still present:

```text
success: false
message: cannot reset intervention while unsafe: stm_fault_mask_nonzero
```

The safety supervisor process and service remained present:

```text
/amr_safety_supervisor
/amr/safety_supervisor/reset_intervention
/amr/safety_supervisor/status publisher count: 1
```

The long probe that spanned the fault saw no communication instability:

```text
wheel: 9.742 Hz
fault: 9.709 Hz
diag: 1.992 Hz
odom: 49.979 Hz
scan: 9.667 Hz
comm: 2.001 Hz
big_gaps: 0
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
uart_write_failures: 0
uart_write_timeouts: 0
pub_failures: 0
baseline_result: FAIL
  FAIL: fault_mask is nonzero: 24
```

The probe failure was expected because the STM fault was intentionally active.

## Recovery Sequence

After motor-driver power was restored, recovery was performed without restarting the safety supervisor:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: false}"

ros2 topic pub --once /amr_stm/clear_fault std_msgs/msg/Empty "{}"

ros2 service call /amr/safety_supervisor/reset_intervention std_srvs/srv/Trigger "{}"

ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: true}"
```

After `/amr_stm/clear_fault`, STM reported:

```text
fault_mask: 0
safety_state: 65536
comm_status: stm_link_ok
comm_fault_mask: 0
```

`safety_state=65536` is STM `IDLE` with no fault bits while enable is still false.

The supervisor reset then succeeded:

```text
success: true
message: intervention reset
```

STM was manually re-enabled afterward.

## Post-Recovery Validation

Final safety status:

```text
mode: enforce
healthy: true
intervention_active: false
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
safety_state: ENABLED
```

Post-recovery 30 second probe:

```text
wheel: 9.761 Hz
fault: 9.759 Hz
diag: 1.978 Hz
odom: 50.006 Hz
scan: 9.671 Hz
comm: 1.999 Hz
big_gaps: 0
fault_mask: 0
safety_state: 131072
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
uart_write_failures: 0
uart_write_timeouts: 0
pub_failures: 0
baseline_result: PASS
```

## Findings

- Real motor-driver power loss still produces the expected STM stall fault bits.
- STM communication remained healthy during the fault and recovery.
- The safety supervisor remained in the ROS graph and kept its reset service available.
- Reset was rejected while unsafe and succeeded only after the STM fault was cleared.
- STM re-enable remained manual; the reset service did not re-enable motion.
- Recovery no longer requires restarting the safety supervisor.

## Next Step

Step 11 should add operator-facing recovery tooling around this validated sequence. The goal is a single guarded recovery command or script that:

- Cancels active Nav2/mission motion.
- Publishes zero velocity.
- Disables STM.
- Clears STM faults only after operator confirmation that the physical fault source is gone.
- Calls `/amr/safety_supervisor/reset_intervention`.
- Leaves STM re-enable as an explicit final operator action.

# Safety Step 7: Moving Intervention Validation

Step 7 validates that the enforced safety supervisor can stop the AMR while it is moving. The goal is to prove the active stop path under real wheel output, then recover manually without a power cycle.

Auto re-enable remains disabled for this step.

## Preconditions

- AMR on clear floor.
- Normal enforced supervisor running.
- STM enabled.
- No active mission.
- Pre-test safety status healthy.
- Pre-test baseline probe passes.

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

## Motion Threshold Check

A first check at `0.02 m/s` did not produce wheel output. The command was visible on `/diff_drive_controller/cmd_vel_unstamped`, but duty and wheel velocity stayed at zero. This is not useful for moving-stop validation.

A second check at `0.06 m/s` produced real wheel output:

```text
cmd_linear_mps: 0.060..0.060
duty_left: 0.000..0.129
duty_right: 0.000..0.180
wheel_velocity_abs_radps: 0.000..0.874
baseline_result: PASS
```

Use `0.06 m/s` or higher for future low-speed moving safety tests.

## Stale-Odom Attempt

The Step 6 stale-odom method was attempted during motion by running a temporary supervisor with:

```bash
-p odom_topic:=/test_safety/missing_odom
```

The supervisor intervention fired correctly, but the AMR did not produce wheel output before intervention even when `/amr_stm/enable=true` was explicitly published after the temporary supervisor started:

```text
intervention_reasons: [stale_odom]
safety_state: IDLE
fault_mask: 0
comm_status: stm_link_ok
cmd_linear_mps: 0.000..0.060
duty_left: 0.000..0.000
duty_right: 0.000..0.000
wheel_velocity_abs_radps: 0.000..0.000
baseline_result: PASS
```

Do not use this stale-odom remap method as the primary moving-stop validation. It remains useful for stationary intervention validation only.

## Moving Fault-Mask Injection

The successful moving-stop test used the normal enforced supervisor and a short synthetic ROS-level fault-mask burst while the robot was moving:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.06}, angular: {z: 0.0}}"

ros2 topic pub -r 20 /amr_stm/fault_mask std_msgs/msg/Int32 "{data: 1}"
```

The synthetic `fault_mask=1` sample represents `ESTOP` at the ROS topic level only. It is not a real STM firmware fault; the real STM publisher returned to `fault_mask=0` after the burst.

Observed supervisor intervention:

```text
healthy: false
intervention_active: true
intervention_count: 1
intervention_reasons: [stm_fault_mask_nonzero]
last_intervention_reasons: [stm_fault_mask_nonzero]
fault_mask: 1
faults: [ESTOP]
comm_status: stm_link_ok
comm_fault_mask: 0
```

Probe during the moving intervention:

```text
cmd_linear_mps: 0.000..0.060
duty_left: 0.000..0.147
duty_right: 0.000..0.142
wheel_velocity_abs_radps: 0.000..0.819
fault_mask: 0
safety_state: 65536
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
baseline_result: PASS
```

This confirms the supervisor stopped the AMR after real wheel output had occurred. STM ended in `IDLE`, and the real STM fault mask remained clear.

## Recovery

After intervention, publish zero velocity, restart the safety supervisor to clear the latched intervention state, and re-enable STM:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: true}"

ros2 run amr_safety safety_supervisor --ros-args \
  -p enforce:=true \
  -p require_amcl:=false \
  -p auto_reenable_when_safe:=false
```

Post-recovery status:

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

Post-recovery baseline probe:

```text
wheel: 9.769 Hz
fault: 9.769 Hz
diag: 1.971 Hz
odom: 49.490 Hz
scan: 9.570 Hz
comm: 2.017 Hz
big_gaps: 0
fault_mask: 0
safety_state: 131072
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
baseline_result: PASS
```

## Next Step

Step 8 is documented in `docs/safety/safety_supervisor_step8.md`. It validates a real motor-driver power-cut fault during commanded motion and verifies manual recovery after STM latches stall faults.

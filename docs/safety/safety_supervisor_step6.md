# Safety Step 6: Controlled Intervention Validation

Step 6 validates that the enforced safety supervisor can actively stop the AMR when a monitored input becomes unhealthy, without creating an STM fault or requiring a power cycle.

This test is stationary. It intentionally hides odom from a temporary safety supervisor while keeping the real command and STM enable outputs active. The expected result is a supervisor intervention that publishes zero velocity and disables STM through `/amr_stm/enable=false`.

## Preconditions

- AMR stationary.
- Mission server idle.
- STM already enabled.
- Normal safety status healthy before injection.
- Baseline probe passes before injection.

Pre-injection status:

```text
mode: enforce
healthy: true
action_authority: true
intervention_active: false
intervention_count: 0
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
safety_state: ENABLED
```

Pre-injection baseline probe:

```text
baseline_result: PASS
fault_mask: 0
safety_state: 131072
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
```

## Injection Command

Stop the normal supervisor first, then start a temporary supervisor with odom remapped to a missing topic:

```bash
ros2 run amr_safety safety_supervisor --ros-args \
  -p enforce:=true \
  -p require_amcl:=false \
  -p auto_reenable_when_safe:=false \
  -p odom_topic:=/test_safety/missing_odom
```

## Expected Result

After startup grace and stale dwell, the supervisor should report:

```text
healthy: false
intervention_active: true
intervention_reasons: [stale_odom]
last_intervention_reasons: [stale_odom]
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
safety_state: IDLE
```

Observed result:

```text
healthy: false
intervention_active: true
intervention_count: 124
intervention_reasons: [stale_odom]
last_intervention_reasons: [stale_odom]
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
safety_state: 65536
safety_state.control_state: IDLE
```

This confirms the supervisor can remove STM enable authority without causing an STM fault.

## Recovery

Stop the temporary supervisor, re-enable STM, and restart the normal enforced supervisor:

```bash
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
action_authority: true
intervention_active: false
intervention_count: 0
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
safety_state: ENABLED
```

Post-recovery 15 second baseline probe:

```text
wheel: 9.785 Hz
fault: 9.648 Hz
diag: 1.982 Hz
odom: 49.988 Hz
scan: 9.669 Hz
comm: 2.001 Hz
big_gaps: 0
fault_mask: 0
safety_state: 131072
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
baseline_result: PASS
```

## Next Step

Step 7 is documented in `docs/safety/safety_supervisor_step7.md`. It validates intervention during real low-speed wheel output using a synthetic ROS-level STM fault-mask burst, followed by manual recovery and a post-recovery baseline probe.

# Safety Step 5: Enforced Healthy-Path Validation

Step 5 validates the Step 4 active safety path on hardware while all inputs are healthy. This is not fault injection. The goal is to prove that enabling action authority does not interfere with normal operation.

## Launch Modes

The navigation script starts the safety supervisor in monitor-only mode by default:

```bash
./scripts/open_amr_devpc_navigation.sh my_new_map
```

To explicitly start the safety supervisor with action authority:

```bash
AMR_SAFETY_ENFORCE=true ./scripts/open_amr_devpc_navigation.sh my_new_map
```

AMCL is optional by default, so hardware-only checks do not trigger intervention when localization is not running or when an old `/amcl_pose` sample is present. For full navigation safety validation:

```bash
AMR_SAFETY_ENFORCE=true AMR_SAFETY_REQUIRE_AMCL=true ./scripts/open_amr_devpc_navigation.sh my_new_map
```

Auto re-enable remains disabled in the launch script.

## Healthy-Path Pass Criteria

With enforced mode active:

- `/amr/safety_supervisor/status` reports `mode: enforce`.
- `action_authority` is `true`.
- `healthy` is `true`.
- `intervention_active` is `false`.
- `intervention_count` remains `0`.
- `/amr_stm/fault_mask` remains `0`.
- `/amr_stm/comm_status` remains `stm_link_ok`.
- `/amr_stm/comm_fault_mask` remains `0`.
- Baseline probe passes during a short movement.

## Validation Performed

After navigating to `home`, enforced mode was started manually while the AMR was healthy:

```bash
ros2 run amr_safety safety_supervisor --ros-args \
  -p enforce:=true \
  -p require_amcl:=false \
  -p auto_reenable_when_safe:=false
```

Status before motion:

```text
mode: enforce
healthy: true
action_authority: true
intervention_active: false
intervention_count: 0
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
```

A short low-speed forward motion was run with enforced mode active. The baseline probe passed:

```text
fault_mask: 0
comm_status: stm_link_ok
comm_fault_mask: 0
boot_drops: 0
cmd_linear_mps: 0.000..0.035
current_left_ma: -8..484
current_right_ma: -140..418
baseline_result: PASS
```

Status after motion:

```text
mode: enforce
healthy: true
action_authority: true
intervention_active: false
intervention_count: 0
```

## Next Step

Step 6 should perform controlled fault-injection tests one condition at a time, starting with non-motion conditions such as a deliberately stale test input with outputs remapped, then progressing to carefully controlled hardware faults only when the recovery procedure is ready.

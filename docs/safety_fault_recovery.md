# AMR Safety Fault Decode And Recovery

This document is the Step 2 safety reference. It defines how to decode STM fault masks, communication fault masks, and the current recovery procedure before adding higher-level safety layers.

## STM Control State

`/amr_stm/safety_state` packs two values:

- upper 16 bits: STM control state
- lower 16 bits: STM fault mask

Control states:

| Value | State |
| --- | --- |
| 0 | INIT |
| 1 | IDLE |
| 2 | ENABLED |
| 3 | FAULT |

Decode a live value:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && timeout 5 ros2 topic echo /amr_stm/safety_state'
python3 scripts/amr_decode_faults.py --safety-state 131072
```

`131072` is `0x00020000`, which means `ENABLED` with no STM fault bits.

## STM Fault Mask

`/amr_stm/fault_mask` is latched by STM firmware. A nonzero value means the STM entered `CTRL_STATE_FAULT`.

| Bit | Mask | Name | Meaning |
| --- | --- | --- | --- |
| 0 | `0x0001` | `ESTOP` | Hardware or software e-stop active |
| 1 | `0x0002` | `OC_LEFT` | Left current magnitude above threshold for dwell |
| 2 | `0x0004` | `OC_RIGHT` | Right current magnitude above threshold for dwell |
| 3 | `0x0008` | `STALL_LEFT` | Left wheel commanded/driven but RPM stayed low |
| 4 | `0x0010` | `STALL_RIGHT` | Right wheel commanded/driven but RPM stayed low |
| 5 | `0x0020` | `ENC_TIMEOUT_LEFT` | Left command active but encoder remained idle |
| 6 | `0x0040` | `ENC_TIMEOUT_RIGHT` | Right command active but encoder remained idle |
| 7 | `0x0080` | `ADC_STUCK` | Current ADC appeared stuck or railed while active |
| 15 | `0x8000` | `GENERIC` | Reserved generic firmware fault |

Decode a live value:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && timeout 5 ros2 topic echo /amr_stm/fault_mask'
python3 scripts/amr_decode_faults.py --fault-mask 24
```

Example: `24` is `0x0018`, which decodes to `STALL_LEFT, STALL_RIGHT`.

## Current STM Thresholds

These thresholds are compiled into `STM/STM_Firmware_AMR_v2/Core/Inc/app_config.h`:

| Check | Current Value |
| --- | --- |
| Overcurrent threshold | `1500 mA` |
| Overcurrent dwell | `50 ms` |
| Stall duty minimum | `8%` |
| Stall RPM maximum | `0.5 RPM` |
| Stall dwell | `500 ms` |
| Encoder timeout command RPM minimum | `0.5 RPM` |
| Encoder timeout dwell | `1000 ms` |
| ADC stuck samples | `30` |
| ADC rail threshold | `5 counts from rail` |
| ADC stuck minimum duty | `2%` |

Overcurrent is evaluated using current magnitude, so both positive and negative sensor polarity can trip the same threshold.

## Communication Fault Mask

`/amr_stm/comm_fault_mask` is published on the Jetson side by `amr_link_watchdog`.

| Bit | Mask | Name | Meaning |
| --- | --- | --- | --- |
| 0 | `0x0001` | `STARTUP_TIMEOUT_WAITING_FOR_WHEEL_STATE` | No wheel state arrived before startup timeout |
| 1 | `0x0002` | `STALE_WHEEL_STATE` | Wheel state stopped arriving after startup |

Decode:

```bash
python3 scripts/amr_decode_faults.py --comm-fault-mask 2
```

Healthy communication is:

```text
/amr_stm/comm_status: stm_link_ok
/amr_stm/comm_fault_mask: 0
```

## Reset Cause Mask

`/amr_stm/ros_diag` index `18` contains the STM reset cause mask captured at boot.

| Bit | Mask | Name | Meaning |
| --- | --- | --- | --- |
| 0 | `0x01` | `BOR` | Brown-out reset |
| 1 | `0x02` | `PIN` | NRST/reset pin reset |
| 2 | `0x04` | `POR/PDR` | Power-on or power-down reset |
| 3 | `0x08` | `SOFT` | Software reset |
| 4 | `0x10` | `IWDG` | Independent watchdog reset |
| 5 | `0x20` | `WWDG` | Window watchdog reset |
| 6 | `0x40` | `LPWR` | Low-power reset |

Launch-time `PIN` or `SOFT` can be expected if the launch script, ST-LINK, or reset button reset the STM. A reset cause becomes a failure when `boot_ms` drops during runtime.

## Recovery Procedure

Use this sequence after any STM fault:

1. Stop motion commands.

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{}"'
```

2. Disable STM motor output.

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: false}"'
```

3. Capture and decode fault state before clearing it.

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && timeout 5 ros2 topic echo /amr_stm/fault_mask'
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && timeout 5 ros2 topic echo /amr_stm/safety_state'
python3 scripts/amr_decode_faults.py --fault-mask <value>
```

4. Fix the physical cause.

Do not clear the fault until the cause is gone. Firmware latches new fault bits before processing `clear_fault`, so a fault that is still physically active will immediately remain latched.

5. Clear the STM fault.

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && ros2 topic pub --once /amr_stm/clear_fault std_msgs/msg/Empty "{}"'
```

6. Confirm the mask is zero.

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && timeout 5 ros2 topic echo /amr_stm/fault_mask'
```

7. Re-enable STM motor output.

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: true}"'
```

8. Run a short baseline probe before resuming navigation.

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && python3 ../scripts/amr_baseline_probe.py --duration 20'
```

## Fault-Specific First Checks

| Fault | First checks |
| --- | --- |
| `ESTOP` | E-stop/reset wiring, software e-stop command, loose reset/NRST contact |
| `OC_LEFT`, `OC_RIGHT` | Wheel obstruction, motor driver, wiring short, current calibration, threshold realism |
| `STALL_LEFT`, `STALL_RIGHT` | Wheel jam, motor output wiring, insufficient torque, floor contact, PWM/direction path |
| `ENC_TIMEOUT_LEFT`, `ENC_TIMEOUT_RIGHT` | Encoder wiring, encoder polarity, timer channel, wheel command path |
| `ADC_STUCK` | Current sense wiring, ADC rail, zero calibration, sensor power |
| Communication fault | USB/serial cable, micro-ROS agent, STM power stability, `/amr_stm/wheel_state` rate |

## Step 2 Exit Criteria

- Fault masks can be decoded from docs or `scripts/amr_decode_faults.py`.
- The operator captures the fault before clearing it.
- Clear/re-enable sequence is documented and repeatable.
- STM overcurrent applies to current magnitude, not only positive current polarity.

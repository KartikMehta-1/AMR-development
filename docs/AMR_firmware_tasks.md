# AMR Firmware Tasks (Motor Control)

Owner: Kartik Mehta  
Status: In progress - dual-motor speed PI tuning; cascaded current loop deferred until higher-accuracy current sensor.  
Last Updated: 2025-12-09

Legend: Done, In Progress, Partial, Planned, Not Started, Blocked

## Phase 0 - Bring-up (baseline)
- Status: Done
- Tasks
  - Done - Create `STM_Firmware_AMR_v2` scaffold; build, flash, UART banner
  - Done - Configure TIM1 at 20 kHz (CH1=PA8, CH2=PA9)
  - Done - Left motor constant duty (10%) using MDD20A; PB4 DIR
  - Done - Direction GPIO configured (PB4/PB5)

## Phase 0.5 - Dual-motor PWM/DIR (duty only)
- Status: Done
- Tasks
  - Done - Wire right motor to M2 PWM (PA9) and DIR (PB5)
  - Done - Verify duty sweep 0-20% on both channels with correct directions
  - Done - Confirm common ground and motor power path via E-stop/fuse

## Phase 1 - Encoders online
- Status: Done
- Tasks
  - Done - Start TIM3 (left) and TIM2 (right) encoder interfaces
  - Done - Read counts and RPM on UART (both wheels); direction sign validated and corrected
  - Planned - Add external 3.3 V pull-ups (4.7-10 kOhm) on A/B; optionally enable GPIO pull-ups
  - Planned - Add software invert flag if wiring phase cannot be changed

## Phase 2 - Current sensing online
- Status: Done
- Tasks
  - Done - Start ADC1 scanning channels 8 (PB0) and 11 (PC1)
  - Done - Implement scaling (divider ratio) and zero-offset removal + LPF/averaging; polarity fix
  - Done - Stream raw/filtered current via UART for validation
  - Note: Current kept for telemetry and protection, not used in control loop

## Phase 3 - Control ticks and scheduling
- Status: Partial
- Tasks
  - In Progress - Outer loop now on fixed 100 Hz TIM4 tick with speed PI, duty ramp, and setpoint toggling
  - Planned - Telemetry decimation and timing cleanup as needed
  - Blocked - Inner tick (1-5 kHz) tied to PWM/ADC for current control (deferred until higher-accuracy current sensor)
  - Blocked - Scheduler items tied to inner current loop (ADC sample/filter + current PI + duty) deferred with cascade

## Phase 4 - Single-loop speed control polish
- Status: In Progress
- Tasks
  - In Progress - Tune speed PI gains both wheels; adjust ramp rate and polarity as needed
  - Planned - Add feedforward/skew compensation if needed to balance wheels
  - Planned - Collect and plot step/ramp responses; record rise/settling/overshoot/SSE metrics

## Phase 5 - Safety manager and e-stop
- Status: Planned
- Tasks
  - Planned - Add e-stop GPIO input and debounce
  - Planned - Safety gate: force PWM=0 unless safe and enabled
  - Planned - Latching FAULT with manual clear flow

## Phase 6 - Faults (per firmware_motor_control.md)
- Status: Planned
- Tasks
  - Planned - Overcurrent L/R with time filter and hysteresis
  - Planned - Encoder timeout L/R based on commanded speed
  - Planned - ADC range/stuck detection
  - Planned - Supply under/over-voltage (when available)
  - Planned - Fault mask and telemetry bits

## Phase 7 - Differential drive and tuning
- Status: Done
- Tasks
  - Done - Map (v, omega) to wheel RPM setpoints using geometry; ramp/coordination added
  - Done - Curvature-preserving duty saturation with shared scaling
  - Planned - Tune gains and document results in `docs/pid.md`

## Deferred - Cascaded current + speed control (requires accurate current sensor)
- Status: Blocked
- Tasks
  - Implement PI current controller (anti-windup, clamps) once sensor upgraded
  - Add inner tick (1-5 kHz) tied to PWM/ADC; i_ref handoff from outer loop
  - Outer PI/PID generating i_ref and comparison vs single-loop
  - Telemetry: control terms (w_err, w_p, w_i, w_d) and current terms (i_cmd/i_meas)
  - Run comparison tests and document in `docs/cascaded_pid.md` (results section)

Notes
- See `docs/firmware_motor_control.md` for control architecture, timing, and state machine.
- Thresholds and clear criteria for faults are listed in the same document.
- For mechanical layout and assembly planning, see `docs/mechanical_cad_tasks.md` and the `CAD/` folder structure.
- Hardware support (buck + perfboard) to unblock sensing:
  - Add regulated 5 V buck from 12 V bench/battery for sensors/encoders/ACS758; tie grounds common.
  - Build a small perfboard/shield for Nucleo: encoder 3.3 V pull-ups, ACS758 dividers + RC filters, decoupling caps, screw terminals/headers for clean wiring.
- Schedule cross-ref (see `docs/AMR_project.md`): W20 Mechanical Design Finalization; W21 Electrical Design & Battery AMR (battery operation + robust harness); W22 Dockerized Workspace; W23 ROS2 topics bring-up/validation (cmd_vel/wheel_state/LiDAR/depth/proximity).

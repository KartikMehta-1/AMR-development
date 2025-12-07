# AMR Firmware Tasks (Motor Control)

Owner: Kartik Mehta  
Status: In progress — dual-motor speed PI tuned; current used for telemetry/protection.  
Last Updated: 2025-11-05

Legend: Done, In Progress, Partial, Planned, Not Started

## Phase 0 — Bring-up (baseline)
- Status: Done
- Tasks
  - Done — Create `STM_Firmware_AMR_v2` scaffold; build, flash, UART banner
  - Done — Configure TIM1 at 20 kHz (CH1=PA8, CH2=PA9)
  - Done — Left motor constant duty (10%) using MDD20A; PB4 DIR
  - Done — Direction GPIO configured (PB4/PB5)

## Phase 0.5 — Dual-motor PWM/DIR (duty only)
- Status: Done
- Tasks
  - Done — Wire right motor to M2 PWM (PA9) and DIR (PB5)
  - Done — Verify duty sweep 0-20% on both channels with correct directions
  - Done — Confirm common ground and motor power path via E-stop/fuse

## Phase 1 — Encoders online
- Status: Done
- Tasks
  - Done — Start TIM3 (left) and TIM2 (right) encoder interfaces
  - Done — Read counts and RPM on UART (both wheels); direction sign validated and corrected
  - Planned — Add external 3.3 V pull-ups (4.7-10 kOhm) on A/B; optionally enable GPIO pull-ups
  - Planned — Add software invert flag if wiring phase cannot be changed

## Phase 2 — Current sensing online
- Status: Done
- Tasks
  - Done — Start ADC1 scanning channels 8 (PB0) and 11 (PC1)
  - Done — Implement scaling (divider ratio) and zero-offset removal + LPF/averaging; polarity fix
  - Done — Stream raw/filtered current via UART for validation
  - Note: Current kept for telemetry and protection (fault thresholds TBD), not used in control loop

## Phase 3 — Control ticks and scheduling (cascaded)
- Status: Partial
- Tasks
  - In Progress — Outer loop running in main tick (~10 Hz) with speed PI, duty ramp, and setpoint toggling for test plots
  - Planned — Add base timer (100-200 Hz) for speed loop and telemetry decimation
  - Planned — Add inner tick (1-5 kHz) tied to PWM/ADC for future current control
  - Planned — Main scheduler: inner (ADC sample/filter + current PI + duty), outer (encoders + speed + speed PI/PID + i_ref)

## Phase 4 — Inner current loop (left)
- Status: Planned
- Tasks
  - Planned — Implement PI current controller (anti-windup, clamps)
  - Planned — Calibrate ACS758 offset and scaling; apply moving average/LPF
  - Planned — Acceptance: stable current steps with limited overshoot and bounded duty

## Phase 5 — Cascaded speed control (outer) + right wheel
- Status: Partial
- Tasks
  - In Progress — Single-loop speed PI on both wheels with ramp and polarity fix; target toggling and plotting for tuning
  - Planned — Outer PI/PID generating i_ref for inner loop (deferred until current loop added)
  - Planned — Acceptance: <=10% overshoot, low SSE on step and ramp (single-loop metrics pending)

## Phase 6 — Comparison & Telemetry v2
- Status: Partial
- Tasks
  - In Progress — Telemetry includes RPM targets, duties, currents; live plot shows targets vs actual
  - Planned — Add control-term telemetry (w_err, w_p, w_i, w_d; i_cmd/i_meas if inner loop added)
  - Planned — Run step/ramp/load tests: single-loop vs cascaded; log CSVs and generate plots (`python_scripts/plot_step_compare.py`)
  - Planned — Archive logs in `Workspace/logs/` and plots in `docs/figures/`
  - Planned — Summarize results (rise, overshoot, settling, SSE) in `docs/cascaded_pid.md` under a Results section

## Phase 7 — Safety manager and e-stop
- Status: Planned
- Tasks
  - Planned — Add e-stop GPIO input and debounce
  - Planned — Safety gate: force PWM=0 unless safe and enabled
  - Planned — Latching FAULT with manual clear flow

## Phase 8 — Faults (per firmware_motor_control.md)
- Status: Planned
- Tasks
  - Planned — Overcurrent L/R with time filter and hysteresis
  - Planned — Encoder timeout L/R based on commanded speed
  - Planned — ADC range/stuck detection
  - Planned — Supply under/over-voltage (when available)
  - Planned — Fault mask and telemetry bits

## Phase 9 — Differential drive and tuning
- Status: Planned
- Tasks
  - Planned — Map (v, omega) to (left, right); saturation and ramp coordination
  - Planned — Tune gains; document results in `docs/pid.md`

Notes
- See `docs/firmware_motor_control.md` for control architecture, timing, and state machine.
- Thresholds and clear criteria for faults are listed in the same document.
- For mechanical layout and assembly planning, see `docs/mechanical_cad_tasks.md` and the `CAD/` folder structure.
- Hardware support (buck + perfboard) to unblock sensing:
  - Add regulated 5 V buck from 12 V bench/battery for sensors/encoders/ACS758; tie grounds common.
  - Build a small perfboard/shield for Nucleo: encoder 3.3 V pull-ups, ACS758 dividers + RC filters, decoupling caps, screw terminals/headers for clean wiring.

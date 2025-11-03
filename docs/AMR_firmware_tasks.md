# AMR Firmware Tasks (Motor Control)

Owner: Kartik Mehta
Status: In progress — building from single-motor bring-up to dual-motor closed loop with safety and faults.
Last Updated: 2025-11-03

Legend: Done, In Progress, Planned, Not Started

## Phase 0 — Bring-up (baseline)
- Status: Done
- Tasks
  - Done — Create `STM_Firmware_AMR_v2` scaffold; build, flash, UART banner
  - Done — Configure TIM1 at 20 kHz (CH1=PA8, CH2=PA9)
  - Done — Left motor constant duty (10%) using MDD20A; PB4 DIR
  - Done — Direction GPIO configured (PB4/PB5)

## Phase 1 — Encoders online
- Status: Planned
- Tasks
  - Planned — Start TIM3 (left) and TIM2 (right) encoder interfaces
  - Planned — Read counts and verify direction; add simple RPM calc
  - Planned — Add basic input digital filtering (confirm IC filter values)

## Phase 2 — Current sensing online
- Status: In Progress
- Tasks
  - Planned — Start ADC1 scanning channels 8 (PB0) and 11 (PC1)
  - Planned — Implement scaling (divider ratio) and zero-offset removal
  - Planned — Stream raw/filtered current via UART for validation

## Phase 3 — Control ticks and scheduling (cascaded)
- Status: Planned
- Tasks
  - Planned — Add inner tick (1–5 kHz) tied to PWM/ADC for current control
  - Planned — Add outer tick (100–200 Hz) for speed control and telemetry decimation
  - Planned — Main scheduler: inner (ADC sample/filter → current PI → duty), outer (encoders → speed → speed PI/PID → i_ref)

## Phase 4 — Inner current loop (left)
- Status: Planned
- Tasks
  - Planned — Implement PI current controller (anti-windup, clamps)
  - Planned — Calibrate ACS758 offset and scaling; apply moving average/LPF
  - Planned — Acceptance: stable current steps with limited overshoot and bounded duty

## Phase 5 — Cascaded speed control (outer) + right wheel
- Status: Planned
- Tasks
  - Planned — Outer PI/PID speed loop generates i_ref (±Imax), feeds inner loop
  - Planned — Duplicate for right wheel; synchronize updates
  - Planned — Acceptance: ≤10% overshoot, low SSE on step and ramp

## Phase 6 — Comparison & Telemetry v2
- Status: Planned
- Tasks
  - Planned — Add telemetry fields: i_cmd, i_meas, i_err, i_p, i_i, i_out, duty; w_cmd, w_meas, w_err, w_p, w_i, w_d
  - Planned — Run step/ramp/sine/load tests: single-loop vs cascaded; log CSVs
  - Planned — Generate plots and metrics using `python_scripts/plot_step_compare.py`
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

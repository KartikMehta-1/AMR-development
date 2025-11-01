# AMR Firmware Tasks (Motor Control)

Owner: Kartik Mehta
Status: In progress — building from single-motor bring-up to dual-motor closed loop with safety and faults.
Last Updated: TBD

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
- Status: Planned
- Tasks
  - Planned — Start ADC1 scanning channels 8 (PB0) and 11 (PC1)
  - Planned — Implement scaling (divider ratio) and zero-offset removal
  - Planned — Stream raw/filtered current via UART for validation

## Phase 3 — Control tick and scheduling
- Status: Planned
- Tasks
  - Planned — Add 1 kHz control tick (SysTick or TIM base)
  - Planned — Main-loop scheduler: sample ADC, read encoders, compute velocity
  - Planned — Decimate telemetry to ~50 Hz

## Phase 4 — Speed control (left)
- Status: Planned
- Tasks
  - Planned — PI speed controller for left wheel (anti-windup, clamps)
  - Planned — Direction from setpoint sign; duty from magnitude
  - Planned — Acceptance: tracks step/ramp without oscillation

## Phase 5 — Speed control (right) + sync
- Status: Planned
- Tasks
  - Planned — Duplicate PI for right wheel; synchronize updates
  - Planned — Verify straight-line and in-place turn basics

## Phase 6 — Safety manager and e-stop
- Status: Planned
- Tasks
  - Planned — Add e-stop GPIO input and debounce
  - Planned — Safety gate: force PWM=0 unless safe and enabled
  - Planned — Latching FAULT with manual clear flow

## Phase 7 — Faults (per firmware_motor_control.md)
- Status: Planned
- Tasks
  - Planned — Overcurrent L/R with time filter and hysteresis
  - Planned — Encoder timeout L/R based on commanded speed
  - Planned — ADC range/stuck detection
  - Planned — Supply under/over-voltage (when available)
  - Planned — Fault mask and telemetry bits

## Phase 8 — Telemetry v2
- Status: Planned
- Tasks
  - Planned — CSV fields: t, setpoint, meas, error, u, duty, currents, fault_mask, state
  - Planned — Optional 100 Hz stream with periodic headers

## Phase 9 — Differential drive and tuning
- Status: Planned
- Tasks
  - Planned — Map (v, omega) to (left, right); saturation and ramp coordination
  - Planned — Tune gains; document results in `docs/pid.md`

Notes
- See `docs/firmware_motor_control.md` for control architecture, timing, and state machine.
- Thresholds and clear criteria for faults are listed in the same document.

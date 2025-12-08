# STM32 Firmware Architecture (Motor Control + micro-ROS)

Owner: Kartik Mehta  
Status: In progress — dual-motor speed PI with ramp; current used for telemetry/protection; micro-ROS planned for wheel commands and telemetry.  
Last Updated: 2025-11-06

## Goals
- Deterministic control on STM32: dual-wheel speed PI (single-loop) with smooth ramps.
- Current sensing used for protection/diagnostics (not in the control loop).
- micro-ROS interface for wheel commands (Twist/cmd_vel), enable/estop/clear, and telemetry.
- Safety gating and fault latching.

## Architecture Overview
- **Inputs:** `/cmd_vel` (Twist: linear.x, angular.z), estop/enable/clear commands, hardware E-stop, ADC currents, encoders.
- **Control:** Mode/Safety gate -> speed PI per wheel -> duty ramp -> PWM + DIR.
- **Sensing:** Encoders (TIM3 left, TIM2 right) -> RPM estimator (with LPF); currents (ADC1 CH8/11) -> filtered telemetry and protection thresholds.
- **Actuation:** TIM1 CH1/CH2 PWM @20 kHz, DIR PB4/PB5 to Cytron driver; duty capped at 30%.
- **Telemetry:** USART2 and micro-ROS publish RPM (actual/target), duty, current, state/faults.

## Loop Rates and Ownership
- Inner/current tick: reserved (1–5 kHz) tied to PWM/ADC (not active in control yet).
- Outer/speed loop: 100–200 Hz (currently main tick ~10 Hz; move to base timer). Computes RPM, runs speed PI, updates duty targets.
- Telemetry: 50–100 Hz decimated snapshot.
- ISRs/ticks own control math; RTOS tasks only move messages/buffers.

## Data Flow (single-loop)
1) Receive command (Twist) via micro-ROS; store latest with timestamp.
2) Outer tick: read cmd, map v/ω to left/right RPM targets, clamp, apply ramp, run speed PI → duty targets.
3) Apply duty via TIM1 CH1/CH2 (DIR fixed per polarity).
4) Sense: encoder deltas → RPM (LPF); currents → filtered mA for telemetry/protection.
5) Telemetry: publish RPM, RPM targets, duty, current, state/faults.
6) Safety: estop/fault gates duty to zero; fault latch requires clear.

## Safety and Faults (initial)
- Gate: hardware E-stop, software estop topic, fault latch.
- Proposed faults: overcurrent L/R (filtered threshold + dwell), encoder timeout (cmd > v_min, no pulses), ADC range/stuck, supply under/over-voltage (when available), driver fault pin (if present).
- Clear policy: conditions safe + clear command; return to IDLE then re-enable.

## ROS/micro-ROS Interface
- **Sub** `/cmd_vel` (Twist): linear.x (m/s), angular.z (rad/s); QoS reliable, depth 1–5. Map to wheel RPM via track width and wheel radius.
- **Sub** `/amr/estop`, `/amr/enable`, `/amr/clear_fault` (Bool/Empty or service): reliable.
- **Pub** `/amr/wheel_state` (JointState or custom): position, velocity (RPM), effort=current (optional); best effort 50–100 Hz.
- **Pub** `/amr/safety_state`: state enum + fault_mask; reliable 10–50 Hz and on change.
- Staleness: zero targets if `/cmd_vel` is older than dwell (e.g., 200–500 ms) or agent link lost.

## Parameters (to keep centralized)
- Wheel geometry: track_width, wheel_radius.
- Limits: max_rpm, max_duty (0.30), ramp rate, staleness timeout.
- Speed PI gains per wheel; RPM LPF alpha; duty cap.
- Safety thresholds: overcurrent, encoder timeout, voltage (when available), fault dwell/clear times.

## Current State (as of 2025-11-06)
- PWM/Dir: TIM1 CH1/CH2 @20 kHz; DIR PB4/PB5; duty capped 30%.
- Encoders: TIM3 (left), TIM2 (right); polarity corrected; RPM LPF and duty ramp in use.
- Control: single-loop speed PI both wheels; step toggling for test; gains tuned to reduce oscillation.
- Current sense: ADC1 CH8/11; zero-cal + scaling; filtered; used for telemetry/protection only.
- Telemetry: UART includes RPM (actual/target), duty, current, ADC counts/zeros.
- Cascaded current loop: **deferred**.

## Next Steps
- Move outer speed loop to a base timer (100–200 Hz); keep telemetry decimation.
- Add mailboxes and staleness handling for `/cmd_vel`; integrate enable/estop/clear commands.
- Add safety thresholds (overcurrent/stall) and fault mask/state publication.
- Optional: per-wheel feedforward to reduce duty skew; finalize speed PI gains and log overshoot (<10%) with `python_scripts/check_overshoot.py`.
- Defer cascaded current control unless needed; keep current for protection/logging.

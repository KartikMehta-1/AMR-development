# Cascaded PID Control for AMR Drive

Owner: Kartik Mehta
Status: Proposal + implementation plan
Last Updated: 2025-11-03

Overview
- Goal: Improve disturbance rejection and current limiting by closing an inner motor current (torque) loop beneath the outer speed/position loop.
- Architecture: Inner PI Current loop (fast) + Outer PID Speed loop (slower). Optional Position loop wrapped around Speed for trajectory control.
- Benefits vs. single-loop PID (speed-only):
  - Faster and more stable response to load torque disturbances (inner loop directly regulates torque).
  - Cleaner anti-windup behavior and easier current limiting integration.
  - More consistent dynamics across battery voltage variation and back-EMF changes.

Loop Structure
- Inner Current Loop (PI preferred):
  - Setpoint: current_ref (A) derived from outer speed PID output and optional feedforward.
  - Measurement: motor phase or supply current (ACS758; note bandwidth and sampling constraints).
  - Rate: 1–5 kHz (align with PWM/ADC sampling; at least 5–10× faster than outer loop).
  - Controller: PI with anti-windup and output clamp to PWM duty range.
  - Output: `duty_cmd` (signed PWM + direction), limited by safety gates.
- Outer Speed Loop (PID optional, typically PI):
  - Setpoint: wheel speed (RPM) or rad/s.
  - Measurement: encoder-derived speed.
  - Rate: 100–200 Hz (≥ 5–10× slower than inner loop).
  - Controller: PI or PID. Output: `current_ref` (A), limited to ±I_max.
- Position Loop (optional wrapper):
  - Setpoint: position (counts or radians).
  - Controller: PID generating `speed_ref`, which feeds the speed loop.

Telemetry (proposed v2 fields)
- Inner loop: `i_cmd`, `i_meas`, `i_err`, `i_p`, `i_i`, `i_out` (= duty before clamp) and `duty` (after clamp).
- Outer loop: `w_cmd`, `w_meas`, `w_err`, `w_p`, `w_i`, `w_d`, `i_ref` (alias of `i_cmd`).
- Safety: `fault_mask`, `state`, `estop`.

Tuning Order (recommended)
1) Sensor bring-up & calibration
- Calibrate ACS758 zero-offset at no current; compute scaling (A/LSB).
- Add moving average or low-pass (cutoff well above speed-loop bandwidth, below PWM ripple).

2) Inner current PI
- Plant: approximate as (PWM → current) first-order with delay. Start with conservative gains.
- Loop rate: 1–5 kHz. Validate current step response (with safe current limit and short pulses).
- Anti-windup: back-calculate or clamp integrator; output saturation ±duty_max.

3) Outer speed PI/PID
- Rate: 100–200 Hz with filtered speed measurement.
- Command clamp: map speed error to current_ref with ±I_max limit.
- Tune for ≤10% overshoot, fast rise, minimal steady-state error.

4) Position loop (if needed)
- Slowest loop. Limit generated speed_ref (slew and magnitude) to avoid saturating inner loops.

Safety & Limits
- E-stop: forces duty=0 regardless of loop states.
- Current limit: saturate `i_ref` and/or trigger overcurrent fault via time filter.
- Thermal/voltage: optional derating of `i_ref`.

Comparison Plan (single PID vs cascaded)
- Tests: step, ramp, sine, and load disturbance (e.g., apply friction or incline) at several speeds.
- Metrics: rise time (10–90%), overshoot, 2% settling time, steady-state error (SSE), current peak and RMS.
- Procedure:
  1. Single-loop speed PID (current loop disabled): log CSV with headers including t, w_cmd, w_meas, err, p,i,d, pwm, currents.
  2. Cascaded current+speed: log CSV with added inner-loop fields (i_*).
  3. Use `python_scripts/plot_step_compare.py` to overlay and compute metrics.
- Expected outcome: cascaded shows lower current overshoot, faster recovery from disturbances, and more consistent speed tracking across load changes.

Implementation Notes (STM32 mapping)
- ADC1 DMA: sample ACS758 channels each PWM cycle or at fixed kHz; average N samples per current tick.
- Control tick:
  - Inner tick (e.g., TIM update or DMA half/full complete ISR): update current PI and duty.
  - Outer tick (main loop or base timer): update speed PI/PID and compute `i_ref`.
- Telemetry decimation: downsample to 50–100 Hz to avoid flooding UART.

Open Items
- ACS758 bandwidth and noise characteristics: validate feasible inner loop rate and filter design.
- Driver dynamics (Cytron MDD20A/MDD30C): confirm PWM polarity, deadtime, and effective current control authority.
- Position loop need/priority: add when trajectory accuracy requires it.


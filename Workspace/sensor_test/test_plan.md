# Bench Test Definition: ACS758 vs ACS70331

## 1) Goal
Benchmark two current sensors under identical motor-drive conditions:
- `ACS758` (high-current Hall sensor)
- `ACS70331` (higher-sensitivity Hall sensor)

Primary comparison points:
- zero offset stability
- output noise (RMS / peak-to-peak)
- step response and settling
- PWM ripple sensitivity
- repeatability across repeated runs

## 2) Test Hardware
Required:
- Arduino UNO
- DC motor
- L298N motor driver
- PicoScope
- ACS758 module
- ACS70331 module

Optional but recommended for absolute current accuracy:
- low-side shunt + differential amplifier (or differential probe)

## 3) Test Modes
### Mode A: Relative comparison (no shunt)
Use scope + Arduino to compare sensor behavior relative to each other.
- Good for noise and dynamics.
- Not enough for absolute gain error.

### Mode B: Absolute comparison (with shunt reference)
Add a current reference channel (shunt path) and compute sensor gain error.
- Required for true metrology-level benchmarking.

## 4) Data to Log
Arduino CSV columns (from provided sketch):
- `t_ms`
- `phase_idx`
- `phase`
- `duty_pct`
- `direction`
- `adc_acs758`, `adc_acs70331`
- `v_acs758`, `v_acs70331`
- `i_acs758_A`, `i_acs70331_A`
- `zero_acs758`, `zero_acs70331`
- `trig`

PicoScope channels:
- CH-A: ACS758 output
- CH-B: ACS70331 output
- CH-C: Arduino trigger (phase marker)
- CH-D: shunt/amplifier (optional reference)

## 5) Procedure
1. Power-up with motor stopped.
2. Run startup zero calibration (automatic in sketch).
3. Record 30 to 60 s at `0%` duty for noise baseline.
4. Run duty step sequence (forward):
   - `0 -> 20 -> 40 -> 60 -> 80 -> 60 -> 40 -> 20 -> 0`
5. Repeat at least 3 times.
6. Optional: run same sequence in reverse direction.
7. Optional (Mode B): compare sensor current against shunt-derived current.

## 6) Metrics
Compute these for each sensor:
- Zero offset mean and drift (`mV`, equivalent `mA`)
- Idle noise RMS and p-p (`mV` or converted current)
- Rise time (`10% to 90%`) on duty step edges
- Settling time within +/-5% band
- Overshoot/undershoot at step edges
- PWM ripple amplitude at fixed duty points
- Repeatability (std-dev across runs)

If reference current is present:
- Gain error (%) vs reference
- Linearity error (best-fit residual)

## 7) Pass/Fail Guidance (Practical)
- Prefer the sensor with:
  - lower idle noise
  - smaller drift over test duration
  - cleaner step response (lower ringing)
  - lower PWM-coupled ripple
- For cascaded control candidate, low-current resolution and low noise are higher priority than very high current range.

## 8) Safety and Reliability Notes
- Confirm each sensor supply voltage and output range before wiring.
- Keep grounds common.
- Keep sensor signal wires short and away from motor leads.
- Start from low duty, then increase.
- Stop immediately on excess heating, unstable wiring, or resets.

## 9) Output Artifacts to Save
- Raw CSV from Arduino
- PicoScope waveform files
- A short comparison sheet with:
  - chosen sensor
  - reasons (noise, dynamics, robustness)
  - known limits (range, saturation, thermal behavior)

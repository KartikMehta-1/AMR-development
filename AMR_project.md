# Kartik’s AMR Project Tracker (18 Weeks)
**File:** `AMR_project.md`  
**Owner:** Kartik Mehta  
**Last Updated:** 2025-10-28  
**Scope:** STM32 low-level control, Jetson Nano high-level compute, motor drivers, current sensing, FreeRTOS, ROS2 + Gazebo, SLAM & Navigation.

---

## Status Summary
- Overall: On track with tuning in progress
- Progress: 6/18 weeks complete (~33%)
- Current Focus (Week 7): PID tuning & step testing — target ≤5% steady-state error
- Next Focus (Week 8): E-stop input + safety interlock
- Timeline: Flexible (18 weeks is indicative). Prioritize firmware + ROS; Custom PCB is low priority/optional.

Legend: Done (✓), In Progress (◐), Partial (◒), Planned (○), Blocked (■)

---

## Week-by-Week Plan (canonical view)
| Week | Focus | Status | Key Tasks / Notes |
|---:|---|:---:|---|
| 1 | Safety & Tools Setup | ✓ | E-stop path reviewed; fused power path; STM32 toolchain + Blink verified. |
| 2 | UART + Debug (ADC skipped) | ✓ | Serial comms working; pin mapping documented; ADC intentionally skipped at this stage. |
| 3 | PWM + Motor Driver (L298N) | ◒ | PWM verified, motor spins; ramp duty + E-stop integration deferred to PID stage. |
| 4 | Encoder Hookup & Counting | ✓ | Encoder integrated; direction & count validated; stable RPM reading. |
| 5 | RPM Calculation & Telemetry | ✓ | RPM derived from ticks; serial telemetry logging functional. |
| 6 | PID-Based Motor Control (Implementation) | ✓ | PID loop on STM32; ramp limiter; anti-windup; clean control loop. |
| 7 | PID Tuning & Step Testing | ◐ | Tune Kp/Ki/Kd via step tests; record telemetry; target ≤5% SSE. |
| 8 | E-stop Feature & Safety Interlock | ○ | Map E-stop input; debounce; define fault states; immediate PWM cutoff; latching fault with manual clear; telemetry flag. |
| 9 | Integrated Controller Enclosure | ○ | 3D-print for STM32, Jetson, drivers, current sensor, fuses; airflow & serviceability. |
| 10 | Final Drivetrain Migration (Single Channel) | ○ | Integrate final motors, motor driver, and encoders per provided specs; verify PWM freq/polarity, direction logic, encoder CPR/scaling, and initial current limits. |
| 11 | Dual Motor Bring-Up | ○ | Dual driver integration; independent PID; verify linear/turn control. |
| 12 | Code Architecture & Modularization | ○ | Refactor modules: `motor.c`, `encoder.c`, `pid.c`, `telemetry.c`, `safety.c`; prep for FreeRTOS. |
| 13 | FreeRTOS Integration | ○ | Tasks: motor control, safety, telemetry, Jetson comms; validate scheduler timing. |
| 14 | BMS Integration & Power Supervision | ○ | BMS alerts to STM32; pre-charge; fault-based cutoff logic. |
| 15 | Custom PCB (optional, low priority) | ○ | Optional: consolidate MCU, drivers, current sensing, BMS interface, Jetson header. Focus remains on firmware + ROS. |
| 16 | System Validation (RTOS + Safety) | ○ | Validate loop stability, E-stop latency, current-limit behavior. |
| 17 | ROS2 + Gazebo Simulation | ○ | URDF + Gazebo model; wheel plugin; TF validation. |
| 18 | Jetson–STM32 Bridge, SLAM & Navigation | ○ | ROS2 bridge; SLAM + nav demo on Jetson; endurance test. |

> Canonical view rule: If the table and task board ever conflict, the table wins for schedule; task board wins for day-to-day details.

---

## Task Board (granular, checklists)

### Now — Week 7: PID Tuning & Step Testing
- [ ] Design step test protocol
  - Acceptance: Protocol doc includes amplitudes, step intervals, dwell times, and safety thresholds.
- [ ] Implement telemetry packet v2
  - Acceptance: Adds `setpoint`, `measured_rpm`, `error`, `u(t)`, and `fault_flags`; ≤5% packet loss at 100 Hz.
- [ ] Baseline response capture
  - Acceptance: 5 runs each at three setpoints (low/mid/high); saved CSV with timestamps; reproducible within ≤5%.
- [ ] Fit 1st/2nd order model from data
  - Acceptance: Identified parameters with residuals < 10% RMSE.
- [ ] Tune Kp/Ki/Kd (Ziegler–Nichols + fine tune)
  - Acceptance: SSE ≤ 5%; overshoot ≤ 10%; 10–90% rise time within spec.
- [ ] Validate ramp limiter + anti-windup
  - Acceptance: No windup in saturation; no E-stop triggers during standard tests.
- [ ] Document tuned gains & rationale
  - Acceptance: `docs/pid.md` updated with plots and chosen gains; commit linked.

### Next — Week 8: E-stop Feature & Safety Interlock
- [ ] Hardware input mapping
  - Acceptance: E-stop input pin selected; schematic/wiring updated; pull-up/down strategy documented.
- [ ] Debounce/filtering
  - Acceptance: Debounce implemented (HW or SW) with timing documented; no false triggers in bench tests.
- [ ] Immediate PWM cutoff path
  - Acceptance: On E-stop assert, PWM duty = 0 within specified latency; motors coast/brake as designed.
- [ ] Latching fault + manual clear
  - Acceptance: Fault latches until explicit clear via button/UART; safe re-enable sequence documented.
- [ ] Telemetry + fault flags
  - Acceptance: `fault_flags` includes E-stop bit; events logged in CSV.
- [ ] Safety test report
  - Acceptance: Pass/fail table and logs attached in `docs/safety.md`.

### Prep — Drivetrain Migration (specs + planning)
- [ ] Collect final component specs
  - Motors: nominal voltage, free speed, stall/cont current, gear ratio, KV/Kt if available
  - Encoders: type (quadrature/ABI), CPR/PPR, voltage levels, index channel
  - Driver: model, control interface (PWM+DIR), logic levels, braking behavior, max PWM freq
  - Power: battery voltage, peak current budget, wiring constraints
- [ ] Firmware parameter mapping
  - Acceptance: Encoder CPR → `COUNTS_PER_REV`; PWM freq/prescaler; polarity mapping; safe current limits; ramp rates
- [ ] Bench validation plan
  - Acceptance: Checklists for spin test, direction, RPM scaling, saturation behavior, thermal checks

### Upcoming Highlights (Weeks 9–13)
- [ ] Enclosure CAD (Week 9): board mounts, airflow, cable strain relief; print & test-fit.
- [ ] Final drivetrain migration (Week 10): integrate final motors, motor driver, and encoders; verify PWM freq/polarity, direction, encoder CPR and scaling.
- [ ] Dual-motor bring-up (Week 11): independent PID; linear/turn kinematics sanity.
- [ ] Modular refactor (Week 12): extract drivers/modules; unit tests; CI build.
- [ ] FreeRTOS tasks (Week 13): task periods, priorities, watchdog, timing validation.

### Final Stretch (Weeks 14–18)
- [ ] BMS + power supervision (Week 14): pre-charge, fault matrix, brownout behavior.
- [ ] Custom PCB (Week 15): schematic, layout, DRC/ERC, fab order, bring-up checklist.
- [ ] System validation (Week 16): latency & stability tests; regression pack.
- [ ] ROS2 + Gazebo sim (Week 17): URDF, wheel plugin, TF tree; sim–real parity notes.
- [ ] Jetson–STM32 bridge + SLAM demo (Week 18): nav stack, maps, endurance run report.

---

## Definitions of Done (per milestone)
- PID tuning (W7): SSE ≤ 5%, overshoot ≤ 10%, no sustained oscillation, documented plots & gains.
- E-stop (W8): Latched E-stop with immediate PWM cutoff; manual clear; telemetry flag; test report.
- Dual-motor (W11): Straight-line < 2 cm drift over 5 m; in-place turn ≤5° error.
- FreeRTOS (W13): Tasks meet deadlines under load; CPU < 70%; no missed watchdog.
- Validation (W16): E-stop latency ≤ 50 ms; current-limit interaction stable under step loads.
- SLAM demo (W18): Successful nav in mapped area for ≥ 15 min without collision or watchdog resets.

---

## Metrics
- Control quality: SSE, overshoot, rise/settling time (from logs)
- Reliability: E-stop latency, fault counts/day, watchdog resets
- Throughput: Control loop jitter, CPU utilization (RTOS)
- Power: Peak & average current draw, brownouts
- Delivery: Lead time per milestone, tasks closed/week

---

## Repo Structure (suggested)
```
/docs
  AMR_project.md               # this file
  pid.md                       # tuning notes & plots
  safety.md                    # E-stop, current limits, fault codes
  wiring/acs758_wiring.pdf     # diagrams & photos
/src
  motor.c, encoder.c, pid.c, telemetry.c, safety.c, ...
/scripts
  log_decode.py, plot_step.py
```

---

## Issues & Backlog (triage labels)
- P0 block progress today
- P1 needed for this milestone
- P2 nice to have / future

```text
[P1] Plot script: generate step response graphs from CSV (rise time, overshoot, SSE)
[P1] Telemetry v2: add fault flags + setpoint
[P2] CLI: live Bode-like sweep tool using chirp
```

---

## Risks & Mitigations
- Current spikes trip supply → Add soft-start/ramp; validate with current logs.
- Encoder noise at high RPM → Add digital filter/debounce; verify with scope.
- Thermal on motor driver → Heatsink & airflow in enclosure; current derating logic.
- RTOS timing regressions → Watchdog + scheduler tests; CI to replay step tests on logs.

---

## Decision Log (append entries)
```
YYYY-MM-DD — Switched to Cytron MD30C
Context: L298N thermal + dropout issues
Options: L298N, MD10C, MD30C
Decision: MD30C for current headroom
Consequences: Redesign mount; add airflow
```

---

## Change Log
- 2025-10-28: Marked Week 2 ADC as skipped; set Week 8 next focus to E-stop feature; added Final Drivetrain Migration step; noted flexible timeline and PCB as optional.

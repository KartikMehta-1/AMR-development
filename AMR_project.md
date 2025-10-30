# Kartik’s AMR Project Tracker (18 Weeks)
**File:** `AMR_project.md`  
**Owner:** Kartik Mehta  
**Last Updated:** 2025-10-28  
**Scope:** STM32 low-level control, Jetson Nano high-level compute, motor drivers, current sensing (ACS758 x2), FreeRTOS, ROS2 + Gazebo, SLAM & Navigation.

---

## Status Summary
- Overall: On track with tuning in progress
- Progress: 6/18 weeks complete (~33%)
- Current Focus (Week 7): Firmware v2 scaffold, PWM @20 kHz, encoder TIM3, ADC current sensing (ACS758) bring-up
- Next Focus (Week 8): E-stop input + safety interlock; current telemetry + basic current limit
- Timeline: Flexible (plan extended beyond 18 weeks). Prioritize firmware + ROS; Custom PCB is low priority/optional.
 - Firmware Branching: v1 (bench, L298N + small encoder) is now frozen; all new work proceeds in v2 (Cytron MDD20A + post-gearbox encoder).

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
| 7 | Firmware v2: Scaffold + Pin Map + Current | ◐ | New project `STM_Firmware_AMR_v2`; TIM1 @ 20 kHz (CH1=PA8 left, CH2=PA9 right); Encoders: TIM3 (PA6/PA7 left), TIM2 (PA0/PA1 right); ADC1 with DMA: PB0=IN8 (left current), PC1=IN11 (right current); UART banner. |
| 8 | Firmware v2: Single Motor + Current Telemetry | ○ | Wire MDD20A M1 PWM/DIR; verify duty sweep and direction; bring-up ACS758 readings on UART; implement immediate PWM cut on E-stop assert. |
| 9 | Firmware v2: Encoder Integration | ○ | Post-gearbox encoder (A/B on PA6/PA7); set `counts_per_rev=2400`; RPM @ 100 Hz with input filtering. |
| 10 | Firmware v2: PID + Ramp + Safety | ○ | Closed-loop speed with anti-windup and ramp; E-stop latch + manual clear; current limits using ACS758 feedback. |
| 11 | Firmware v2: Telemetry v2 | ○ | CSV adds setpoint, meas, error, u(t), fault_flags; periodic headers; optional 100 Hz stream. |
| 12 | Firmware v2: Dual-Motor Bring-Up | ○ | Add second PWM/DIR + encoder; independent PIDs; synchronized 100 Hz control. |
| 13 | Firmware v2: Differential Drive | ○ | Map (v, ω) ↔ (left, right); saturation and ramp coordination; basic tests. |
 | 14 | Firmware v2: Proximity Sensors (HW) | ○ | Select 8x proximity sensors (TBD interface: GPIO/ADC/I2C); wiring, pull-ups, protection; update pin map; bench power budget. |
 | 15 | Firmware v2: Proximity Drivers | ○ | Implement drivers and sampling scheduler for 8 sensors; debouncing/filtering; fault detection; add to telemetry. |
 | 16 | Firmware v2: micro-ROS Bring-up | ○ | Integrate micro-ROS on STM32; define msgs; publish wheel_state/obstacles; subscribe wheel_cmd/estop; stable transport to agent. |
 | 17 | Jetson ROS2 + Agent + Docker | ○ | Set up micro-ROS agent and ROS2 workspace on Jetson; dockerize dev/runtime; compose services; basic end-to-end echo tests. |
 | 18 | ROS2 Architecture & Topics | ○ | Define node graph, topics, QoS, message types; draft test plans (unit/integration/sim); document interfaces for motion, safety, sensors. |
| 19 | URDF Modeling | ○ | Base chassis + wheels URDF; inertia estimates; visual/collision meshes; joint limits; TF tree. |
| 20 | Gazebo Simulation | ○ | Diff-drive plugin tuning; sensor plugins for LiDAR/depth/proximity; sim-worlds; baseline nav in sim. |
| 21 | ROS2 Node Implementation | ○ | Implement nodes per architecture (odometry, safety_monitor, sensor_fusion, teleop); CI for lint/build/test. |
| 22 | System Tests & CI | ○ | Sim integration tests; logging/bagging; performance dashboards; dockerized CI pipeline. |
| 23 | Field Bring-up | ○ | On-robot tests: drive, stop, obstacle detection; telemetry review; safety validation. |
| 24 | Polish & Docs | ○ | User/developer docs; scripts; troubleshooting; backlog triage. |

> Canonical view rule: If the table and task board ever conflict, the table wins for schedule; task board wins for day-to-day details.

---

## Task Board (granular, checklists)

### Now — Firmware v2: Scaffold + Single Motor + Current
- [ ] New project scaffold `STM_Firmware_AMR_v2` with pins/timers/ADC
  - Acceptance: Builds, LED blinks, UART prints profile `final-single-mdd20a`
- [ ] TIM1 PWM @ 20 kHz; Left: PA8/CH1 + PB4 DIR; Right: PA9/CH2 + PB5 DIR
  - Acceptance: 0→100% duty sweep; forward/reverse verified on left (right optional now)
- [ ] ADC1 channels for ACS758 (PB0=IN8 Left, PC1=IN11 Right); divider + RC filter; DMA circular
  - Acceptance: Zero-current near mid-scale; readings change with load in expected direction
- [ ] E-stop immediate cut path (software + acknowledge)
  - Acceptance: PWM forced to 0 on E-stop; telemetry flag set

### Next — Firmware v2: Encoder Integration
- [ ] Post-gearbox encoder on PA6/PA7 with 3.3 V pull-ups
  - Acceptance: Position monotonic; RPM stable at constant duty; CPR=2400 configured

### Then — Firmware v2: PID + Safety
- [ ] PID with anti-windup + ramp limiter
  - Acceptance: Step tests meet initial SSE/overshoot targets; no windup
- [ ] E-stop latch + manual clear; telemetry `fault_flags`
  - Acceptance: Latches until explicit clear; events logged in CSV

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
- [ ] Firmware v2 Telemetry v2 (Week 11): packet fields and 100 Hz option
- [ ] Dual-motor bring-up (Week 12): independent PID; linear/turn kinematics sanity.
- [ ] Differential drive mapping (Week 13): (v, ω) ↔ (L/R) targets

### Firmware v2: Proximity Sensors (Weeks 14–15)
- [ ] Sensor selection + interface
  - Acceptance: Model chosen (GPIO/ADC/I2C); pin map updated; wiring and protection documented
- [ ] Driver + sampling scheduler
  - Acceptance: 8 sensors read at defined rate; filtering/debounce reduces false positives; fault detection for timeouts
- [ ] Obstacle event API
  - Acceptance: Firmware produces per-sensor range/events; UART/micro-ROS telemetry updated

### Firmware v2: micro-ROS Bring-up (Week 16)
- [ ] Build micro-ROS for STM32 (rmw_microxrcedds)
  - Acceptance: App links and runs; agent connectivity proven on serial/UDP transport
- [ ] Topics + messages
  - Acceptance: Publish `/amr/wheel_state` (sensor_msgs/JointState), `/amr/obstacles` (sensor_msgs/Range[] or custom); subscribe `/amr/wheel_cmd` (geometry_msgs/Twist or custom) and `/amr/estop` (std_msgs/Bool)
- [ ] Reliability
  - Acceptance: QoS set; reconnect behavior validated; watchdog stable under nominal load

### Jetson: ROS2 + Agent + Docker (Week 17)
- [ ] micro-ROS agent service
  - Acceptance: systemd or docker-compose managed agent; logs persisted
- [ ] ROS2 workspace (Humble/Iron)
  - Acceptance: Baseline packages build; colcon test passes; udev rules for serial
- [ ] Docker dev env
  - Acceptance: Dockerfiles for desktop/Jetson; compose for agent + ros-core; documented workflow

### ROS2 Architecture & Topics (Week 18)
- [ ] Node graph & topics
  - Acceptance: Documented nodes: `mc_interface`, `odometry`, `safety_monitor`, `sensor_fusion`, `teleop`, `sim_bridge` with topics/QoS and message types
- [ ] Test plans
  - Acceptance: Unit/integration/sim test plans with pass criteria; bagging procedure defined

### URDF + Gazebo (Weeks 19–20)
- [ ] URDF base + xacro
  - Acceptance: Chassis, wheels, sensors; inertials approximated; TF verified
- [ ] Gazebo simulation
  - Acceptance: Diff-drive plugin; sensor plugins (LiDAR/depth/proximity); simple world; teleop in sim

### ROS2 Nodes + CI (Weeks 21–22)
- [ ] Implement nodes
  - Acceptance: Nodes run with expected topics; tests green in CI
- [ ] CI/CD
  - Acceptance: Dockerized CI for build/test/lint; artifacts published; compose for local runs

---

## Firmware v2 Build Sequence (Final Drivetrain)

Rationale: Keep the current bench firmware intact. Create a separate STM32CubeIDE project for the final drivetrain with Cytron MDD20A and post‑gearbox encoder. Reuse modules (`motor.c`, `encoder.c`, `pid.c`, `safety.c`, `telemetry.c`) and parameterize via `docs/pin_map.yaml`.

1) Project scaffold + pin mapping
- [ ] Create new project: `STM_Firmware_AMR_v2` (board: Nucleo‑F401RE)
- [ ] Apply pin map profile: `profile: final-single-mdd20a`
- [ ] TIM1 PWM @ 20 kHz (ARR/PSC set): CH1=PA8 (left), CH2=PA9 (right)
- [ ] Encoders: TIM3 (PA6/PA7 left), TIM2 (PA0/PA1 right) with input filters ≈10
- [ ] ADC1 enable + DMA + channels for ACS758 on `PB0/IN8` (left) and `PC1/IN11` (right)
  - DoD: Builds and blinks; UART telemetry banner prints profile name; ADC zeros close to mid-scale at 0 A

2) Single motor drive (no encoder)
- [ ] Wire MDD20A M1 PWM/DIR; verify 0→100% duty sweep
- [ ] Implement coast/brake policy and E‑stop immediate PWM cut
- [ ] Stream current telemetry for left channel; verify response to load
- DoD: Motor spins both directions; PWM response linear; current signal monotonic with load; E‑stop cuts within spec

3) Encoder integration (post‑gearbox)
- [ ] HN3806 A/B → PA6/PA7 with 3.3 V pull‑ups; set `counts_per_rev: 2400`
- [ ] RPM computation at 100 Hz; add basic low‑pass
- DoD: Position counts monotonic; RPM stable at constant duty

4) PID + ramp + safety
- [ ] PID loop with anti‑windup; setpoint ramp; soft output clamp
- [ ] E‑stop latch + manual clear; telemetry `fault_flags`
- [ ] Current limit: soft foldback when current exceeds threshold; log overcurrent events
- DoD: Step tests meet initial SSE/overshoot targets; no windup or unsafe behavior; current limiting behaves predictably

5) Telemetry v2
- [ ] CSV includes `t,pos,setpoint,meas,err,p,i,d,pwm,fault_flags`
- [ ] Optional 100 Hz stream with periodic headers
- DoD: Logs parse cleanly; <5% packet loss at 100 Hz

6) Dual‑motor bring‑up
- [ ] Add right motor PWM/DIR (TIM1 CH2 or other), second encoder/timer if needed
- [ ] Independent PID loops; synchronized 100 Hz control
- DoD: Straight‑line test with low drift; in‑place turn within angle spec

7) Differential drive mapping
- [ ] (v, ω) → (left, right) wheel targets; saturation and ramp coordination
- DoD: Velocity command tracking behaves as expected in bench tests

8) Odometry (optional at this stage)
- [ ] Integrate wheel odometry; publish over UART for ROS bridge
- DoD: Reasonable pose estimate over short runs without excessive drift

Notes
- Keep v2 project independent of the bench project to de‑risk migration.
- When stable, consider back‑porting improvements or archiving the bench project as reference.

Current Sensing (integration notes)
- See `docs/pin_map.yaml` ADC channels and `docs/wiring_schematic.md` for divider/filter values.
- Confirm ACS758 variant and sensitivity; update firmware constants and documentation once known.

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

---

## ROS2 Architecture (Draft)
- Nodes
  - `mc_interface` (micro-ROS on STM32): subscribes `/amr/wheel_cmd`, `/amr/estop`; publishes `/amr/wheel_state`, `/amr/obstacles`
  - `odometry`: subscribes `/amr/wheel_state`; publishes `/odom`, `/tf`
  - `safety_monitor`: subscribes `/amr/obstacles`, `/amr/estop`; publishes `/amr/safety_state`
  - `sensor_fusion`: fuses proximity/LiDAR/depth; publishes `/amr/obstacles`
  - `teleop` or higher-level commander: publishes `/cmd_vel` or `/amr/wheel_cmd`
  - `sim_bridge`: interfaces Gazebo topics with AMR topics
- Topics (proposed)
  - `/amr/wheel_cmd` (geometry_msgs/Twist or custom wheel velocities)
  - `/amr/wheel_state` (sensor_msgs/JointState)
  - `/amr/obstacles` (sensor_msgs/Range[] or custom)
  - `/amr/estop` (std_msgs/Bool), `/amr/safety_state` (std_msgs/UInt32)
  - `/odom` (nav_msgs/Odometry), `/tf`, `/tf_static`

---

## Docker Development Environment (Draft)
- Images/containers
  - Firmware build: arm-none-eabi toolchain, micro-ROS build tools
  - Jetson runtime: ROS2 base, micro-ROS agent, AMR nodes
  - Desktop dev: ROS2, Gazebo, build/test tools
- Compose
  - `agent`, `ros-core`, `sim`, `tools` services; volumes for logs and bags
- CI integration
  - Build, lint, test stages; multi-arch where needed
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

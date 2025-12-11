# Kartik's AMR Project Tracker (34 Weeks)
**File:** `AMR_project.md`  
**Owner:** Kartik Mehta  
**Last Updated:** 2025-12-09  
**Scope:** STM32 low-level control, Jetson Nano high-level compute, motor drivers, current sensing (ACS758 x2), FreeRTOS, ROS2 + Gazebo, SLAM & Navigation.

---

## Status Summary
- Overall: On track with tuning in progress
- Progress: 10/34 weeks complete (~29%)
- Recent: Dual-wheel speed PI with duty ramp and target toggling; current telemetry calibrated and used for logging/protection; encoder polarity corrected.
- Current Focus: Speed PI tuning and fault thresholds (overcurrent/stall) using current sensing; optional feedforward to reduce duty skew.
- Cascaded current loop: Deferred until higher-accuracy current sensor is integrated.
- Next Focus (Weeks 11-12): Wrap up single-loop speed control plots/metrics; leave cascaded loop deferred.
- Timeline: Flexible (now a 34-week plan with extensions). Prioritize firmware + ROS; custom PCB is low priority/optional.
  - Firmware Branching: v1 (bench, L298N + small encoder) is now frozen; all new work proceeds in v2 (Cytron MDD20A + post-gearbox encoder).

Legend: Done, In Progress, Partial, Planned, Blocked

---

## Week-by-Week Plan (canonical view)
| Week | Focus | Status | Key Tasks / Notes |
|---:|---|:---:|---|
| 1 | Safety & Tools Setup | Done | E-stop path reviewed; fused power path; STM32 toolchain + Blink verified. |
| 2 | UART + Debug (ADC skipped) | Done | Serial comms working; pin mapping documented; ADC intentionally skipped at this stage. |
| 3 | PWM + Motor Driver (L298N) | Partial | PWM verified, motor spins; ramp duty + E-stop integration deferred to PID stage. |
| 4 | Encoder Hookup & Counting | Done | Encoder integrated; direction & count validated; stable RPM reading. |
| 5 | RPM Calculation & Telemetry | Done | RPM derived from ticks; serial telemetry logging functional. |
| 6 | PID-Based Motor Control (Implementation) | Done | PID loop on STM32; ramp limiter; anti-windup; clean control loop. |
| 7 | Firmware v2: Scaffold + Pin Map + Current | Done | New project `STM_Firmware_AMR_v2`; TIM1 @ 20 kHz (CH1=PA8 left, CH2=PA9 right); Encoders: TIM3 (PA6/PA7 left), TIM2 (PA0/PA1 right); ADC1 with DMA: PB0=IN8 (left current), PC1=IN11 (right current); UART banner. |
| 8 | Firmware v2: Dual-Motor Duty Bring-Up | Done | M2 PWM/DIR (PA9/PB5) wired; duty sweep validated both channels; E-stop cut and GND common confirmed. |
| 9 | Firmware v2: Encoder Integration | Done | Encoders online both wheels (TIM3 PA6/PA7 left, TIM2 PA0/PA1 right); UART RPM confirmed; direction corrected; pull-ups to be added. |
| 10 | Firmware v2: Current Telemetry + Calibration | Done | ADC1 scan IN8/IN11; zero-offset + scaling; filtered current stream; current reserved for logging/faults (not in loop). |
| 11 | Firmware v2: Control (Single-Loop PID) | In Progress | Closed-loop speed PI both wheels; duty ramp; polarity fix; target toggling and plotting; continuing gain/feedforward tuning. |
| 12 | Firmware v2: Cascaded Control + Comparison | Blocked | Deferred until higher-accuracy current sensor; stay on single-loop speed control for now. |
| 13 | Firmware v2: Differential Drive | Done | Map (v, I%) -> wheel RPM; ramp/coordination added; saturation with curvature-preserving scaling; basic 5 s test sequence running. |
| 14 | Firmware v2: Proximity Sensors (HW) | Planned | Select 4x proximity sensors (GPIO/ADC/I2C TBD); mounts, wiring, pull-ups/protection; update pin map; bench power budget. |
| 15 | Firmware v2: Proximity Drivers | Planned | Implement drivers and sampling scheduler for 4 sensors; debouncing/filtering; fault detection; add to telemetry. |
| 16 | Firmware v2: micro-ROS Bring-up | Planned | Integrate micro-ROS on STM32; define msgs; publish wheel_state/obstacles; subscribe wheel_cmd/estop; stable transport to agent. |
| 17 | Dev PC Env & Tooling | Planned | Install ROS2 desktop + colcon, VS Code/devcontainer, CLI tools; micro-ROS agent loopback; cross-build toolchain; base Docker/compose aligned with Jetson; SSH keys + dotfiles for reproducible setup. |
| 18 | Jetson Nano ROS2/JetPack | Planned | Flash JetPack (Ubuntu matching dev PC); install ROS2 + micro-ROS agent; enable CUDA; configure services on boot; verify `ros2 topic list` and talker/listener on hardware. |
| 19 | Wireless PC<->Nano | Planned | Add Wi-Fi module/antennas; configure NetworkManager/wpa_supplicant; set static/reserved IP + SSH keys (no passwords); ping/iperf latency check; NTP sync; optional VPN (WireGuard/Tailscale). |
| 20 | Mechanical Design Finalization | Planned | Complete CAD for chassis/enclosures/sensor mounts; wire harness routing/lengths; connectors + strain relief; STEP/DXF + exploded assembly + BOM; serviceability review. |
| 21 | Electrical Design & Battery AMR | Planned | Operate AMR on battery; finalize wiring harness and routing for robustness (strain relief, protection); power tree battery->fuse->switch->bucks; E-stop integration; charger/BMS pick; ground strategy; bench validation under load. |
| 22 | Dockerized Workspace | Planned | Build desktop + Jetson images (Ubuntu+ROS2+micro-ROS agent); JetPack runtime in Jetson containers; compose with volumes for logs/bags; GPU access validated; `ros2 topic echo` across containers over Wi-Fi; helper scripts. |
| 23 | ROS2 Topics Bring-up & Validation | Planned | Create/validate topics: `cmd_vel`/`/amr/wheel_cmd`, wheel_state, LiDAR scan, depth cam image/point cloud, proximity ranges; set QoS; rosbag + playback tests; document message contracts. |
| 24 | URDF Modeling | Planned | Base chassis + wheels URDF; inertia estimates; visual/collision meshes; joint limits; TF tree; sync with mechanical CAD. |
| 25 | Gazebo Simulation | Planned | Diff-drive plugin tuning; sensor plugins for LiDAR/depth/proximity; sim-worlds; baseline nav in sim; align topics/QoS with real robot. |
| 26 | ROS2 Node Implementation | Planned | Implement nodes per architecture (odometry, safety_monitor, sensor_fusion, teleop); CI for lint/build/test. |
| 27 | System Tests & CI | Planned | Sim integration tests; logging/bagging; performance dashboards; dockerized CI pipeline. |
| 28 | Field Bring-up | Planned | On-robot tests: drive, stop, obstacle detection; telemetry review; safety validation. |
| 29 | Polish & Docs | Planned | User/developer docs; scripts; troubleshooting; backlog triage. |
| 30 | PCB Concept & Requirements | Planned | Finalize end-state architecture (STM32 vs SOM, dual motor stage, rails, IO buses); measure/record real currents, noise, harness lengths; write electrical requirements. |
| 31 | Carrier PCB (Dev Modules) | Planned | Design carrier/backplane for Nucleo + Cytron + external buck; connectors, power distribution, current sensing, ferrites/filters, ground planes; fab + bench bring-up. |
| 32 | Custom Motor Driver Integration | Planned | Drop Cytron; design dual H-bridge with gate driver (e.g., DRV87xx) + MOSFETs + shunt/Hall sensing; protection (TVS/fuse/reverse); thermal + EMC checks. |
| 33 | Bare MCU Integration | Planned | Replace Nucleo with bare STM32F401: crystal, boot config, SWD header, decoupling, ESD/TVS/brownout; firmware port and bring-up (clocks/debug/motor control). |
| 34 | EMC & Production Prep | Planned | 4-layer stack, split/stitched grounds, Kelvin/star grounds, common-mode chokes + TVS, test points, silks/labels, panelization; pre-scan EMC/ESD; pilot build (5-10 units). |

> Canonical view rule: If the table and task board ever conflict, the table wins for schedule; task board wins for day-to-day details.

---

## Detailed Tasks
The granular task board has been moved to docs/AMR_firmware_tasks.md to keep this file focused on high-level goals and the weekly schedule.

Status: See the Status Summary above and task-by-task statuses in docs/AMR_firmware_tasks.md.

---

## Definitions of Done (per milestone)
- Dual-motor duty: Both channels drive 0-20% with correct direction and safe stop via E-stop
- Encoders: Stable RPM both wheels; correct direction; noise filtered
- Current telemetry: Calibrated A/LSB; filtered stream at 50-100 Hz
- Control (single-loop): <=10% overshoot, low SSE on step/ramp; plots archived
- Cascaded control: Inner current loop stable; outer speed loop SSE <= 5%, overshoot <= 10%; comparison plots and gains documented
- FreeRTOS (W13): Tasks meet deadlines under load; CPU < 70%; no missed watchdog
- Validation (W16): E-stop latency <= 50 ms; current-limit interaction stable under step loads
- SLAM demo (W18): Successful nav in mapped area for ~15 min without collision or watchdog resets

---

## PCB Migration Milestones (dev boards -> integrated AMR control PCB)
- Conceptual architecture (target end-state): Define final board contents (STM32/bare MCU or SOM, dual motor stage, buck rails 12->5->3.3 V, encoder conditioning, current sense, battery protection, all IO connectors, CAN/UART/RS485/I2C). This is the "Cytron + Nucleo + buck + encoder + UART + IO" rolled into one.
- Phase A - Measure on dev boards: With Nucleo + Cytron + XL4016, capture real currents (continuous/peak), encoder voltage tolerance/noise, UART/CAN bandwidth, EMI/ground noise patterns, ADC resolution needs, harness lengths and connector types. This produces the electrical requirements doc.
- Phase B - Consolidation carrier PCB: Keep Nucleo + Cytron + external buck, but design a carrier/backplane that handles connectors, power distribution, current sensing, ferrite/filter caps, and ground planes to organize wiring and validate signal/power integrity.
- Phase C - Integrate motor driver: Drop Cytron; design your own dual H-bridge with gate driver (e.g., DRV87xx) + MOSFETs + shunts/Hall sensors. Validate thermals, switching, protection (TVS/fuse/reverse), and EMC.
- Phase D - Integrate MCU: Replace Nucleo with bare STM32F401 (LQFP-64): crystal, boot config, SWD header, decoupling, ESD/TVS, brownout protection. Port firmware; bring-up clocks, debug, boot, and motor control.
- Phase E - Production/EMC-ready: Move to 4-layer, split/stitched grounds, Kelvin sensing, star grounds, TVS + common-mode chokes, reverse/fuse protection, panelization notes, silks/labels, test points. Run pre-scan EMC/ESD and pilot build (5-10 boards) with harnesses and acceptance tests.
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
[P1] Mechanical CAD: full AMR layout (chassis, mounts, sensor brackets, harness routing); deliver STEP/DXF + assembly guide
[P2] CLI: live Bode-like sweep tool using chirp
```

---

## Risks & Mitigations
- Current spikes trip supply + Add soft-start/ramp; validate with current logs.
- Encoder noise at high RPM + Add digital filter/debounce; verify with scope.
- Thermal on motor driver + Heatsink & airflow in enclosure; current derating logic.
- RTOS timing regressions + Watchdog + scheduler tests; CI to replay step tests on logs.

---

## Decision Log (append entries)
```
YYYY-MM-DD - Switched to Cytron MD30C
Context: L298N thermal + dropout issues
Options: L298N, MD10C, MD30C
Decision: MD30C for current headroom
Consequences: Redesign mount; add airflow
```

---

## Change Log
- 2025-10-28: Marked Week 2 ADC as skipped; set Week 8 next focus to E-stop feature; added Final Drivetrain Migration step; noted flexible timeline and PCB as optional.
- 2025-11-XX: Started AMR CAD assembly with dual SO101 arm manipulators mounted; initial rough layout committed to repo.
- 2025-11-XX: Added main power switch at pack output ahead of fuse/E-Stop; updated hardware block diagram accordingly.
- 2025-11-XX: Added DSN-DVM/DUM-368 battery voltage display (fed after main switch) to specs and wiring docs.
- 2025-11-XX: Selected 4x CS100A ultrasonic proximity sensors (trig/echo to STM32); updated specs, wiring, and diagram.
- 2025-11-XX: Power split: XH-M401 (XL4016 class) for Jetson/hub 5 V rail; LM2596 for logic/proximity 5 V rail.
- 2025-11-XX: Cascaded current loop deferred; continue single-loop speed PI until higher-accuracy current sensor is added.

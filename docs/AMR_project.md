# Kartik's AMR Project Tracker (42 Weeks)
**File:** `AMR_project.md`  
**Owner:** Kartik Mehta  
**Last Updated:** 2026-04-17  
**Scope:** STM32 low-level control, Jetson Nano high-level compute, motor drivers, current sensing (ACS758 x2), FreeRTOS, ROS2 + Gazebo, SLAM & Navigation; eventual goal is a fully autonomous AMR with dual SO-101 manipulators that can pick/place small objects using state-of-the-art VLA/VLM/LLM-based policies.

---

## Status Summary
- Overall: SLAM + localization + navigation pipeline is working on real hardware (slam_toolbox map -> saved map -> AMCL -> Nav2 goals); SO-101 integration and an initial ACT manipulation demo are now validated; low-level control/odometry calibration is still in active tuning.
- Progress: 20/42 weeks complete (~48%) (plan is now being executed iteratively vs. strictly week-by-week).
- Recent:
  - slam_toolbox bring-up working with real LiDAR (`/scan` -> `/map`) and RViz config saved.
  - Saved a real map and validated map_server + AMCL localization; Nav2 navigation (planner/controller) working with RViz goal tool.
  - Added a Nav2 wrapper launch + params to remap `/cmd_vel` + `/odom` to ros2_control diff-drive topics and to launch RViz reliably inside Docker.
  - Fixed RViz map display QoS (Transient Local + Reliable) and added Nav2 UI tools/panel to `amr.rviz`.
  - Marked Week 21 (URDF base) and Week 22 (mapping/localization/navigation bring-up) complete; EKF fusion is tracked as a separate improvement.
  - Resolved TF bring-up issues by ensuring ros2_control/controller_manager + `/joint_states` are alive before SLAM.
  - LiDAR alignment parameterized in URDF (yaw adjust) and rebuilt into runtime container workflow.
  - Encoder pipeline validated on bench (manual rotations), moved to higher-resolution timer mode (TI12), and began closing the loop on wheel kinematics (track width / wheel separation) using 360-degree tests.
  - Added firmware tuning profiles for controlled A/B testing (launch guard, feedforward/static FF, etc.) to avoid “many knobs at once”.
  - Integrated SO-101 into the active project stack and completed the main AMR mechanical assembly.
  - Ran an ACT policy that successfully picked up an object and placed it into a bag.
- micro-ROS: STM32 bring-up on USART2 with full AMR topic set live (wheel_state + duty topics visible); UART telemetry disabled to avoid contention.
- Safety: Hardware e-stop GPIO integrated with debounce and fault latch.
- Current Focus (next sprint):
  - Integrate proximity sensors on the AMR: finish mounts/wiring and complete bring-up in the sensing stack.
  - Integrate a higher-accuracy current sensor and develop the associated safety/protection algorithm.
  - Mount the camera on SO-101 and refine the AMR-mounted camera/depth-sensor placement for better gripping and bagging viewpoints.
  - Recalibrate camera-to-arm / camera-to-base extrinsics after the sensor moves and revalidate grasp alignment.
  - Continue ACT manipulation iteration while the sensing and safety integrations are in progress.
- Control Architecture Direction:
  - Yes, moving toward cascaded control makes sense: inner current/torque limiting (or current loop if feasible) + outer speed loop is the standard industrial structure and will reduce slip/launch transients once current sensing is reliable.
- Next Focus:
  - Close the loop on traction/launch transients (feedforward + ramp + slip) using current + better odom; then validate Nav2 navigation on battery power for 10-15 minute supervised runs.
- Timeline: Flexible (now a 42-week plan with extensions). Prioritize firmware + ROS; custom PCB is low priority/optional.

## Calendar Baseline (Week Alignment)
- Week 1 start: 2025-10-17.
- Paused weeks (approx): 2025-11-03 to 2025-11-09 and 2025-11-17 to 2025-11-23.
- Plan weeks are working weeks; paused weeks extend the calendar timeline by ~2 weeks. Update these dates if the break windows were different.
  - Firmware Branching: v1 (bench, L298N + small encoder) is now frozen; all new work proceeds in v2 (Cytron MDD20A + post-gearbox encoder).

Legend: <span style="color: green">Done</span>, <span style="color: goldenrod">In Progress</span>, Partial, Planned, Blocked

---

## Week-by-Week Plan (canonical view)
| Week | Focus | Status | Key Tasks / Notes | Target Date | Achieved Date |
|---:|---|:---:|---|---|---|
| 1 | Safety & Tools Setup | <span style="color: green">Done</span> | E-stop path reviewed; fused power path; STM32 toolchain + Blink verified. | 2025-10-23 | 2025-10-17 |
| 2 | UART + Debug (ADC skipped) | <span style="color: green">Done</span> | Serial comms working; pin mapping documented; ADC intentionally skipped at this stage. | 2025-10-30 | 2025-10-18 |
| 3 | PWM + Motor Driver (L298N) | <span style="color: green">Done</span> | PWM verified, motor spins; ramp duty + e-stop integration completed in v2. | 2025-11-13 | 2025-10-18 |
| 4 | Encoder Hookup & Counting | <span style="color: green">Done</span> | Encoder integrated; direction & count validated; stable RPM reading. | 2025-11-27 | 2025-10-22 |
| 5 | RPM Calculation & Telemetry | <span style="color: green">Done</span> | RPM derived from ticks; serial telemetry logging functional. | 2025-12-04 | 2025-10-22 |
| 6 | PID-Based Motor Control (Implementation) | <span style="color: green">Done</span> | PID loop on STM32; ramp limiter; anti-windup; clean control loop. | 2025-12-11 | 2025-10-23 |
| 7 | Firmware v2: Scaffold + Pin Map + Current | <span style="color: green">Done</span> | New project `STM_Firmware_AMR_v2`; TIM1 @ 20 kHz (CH1=PA8 left, CH2=PA9 right); Encoders: TIM3 (PA6/PA7 left), TIM2 (PA0/PA1 right); ADC1 with DMA: PB0=IN8 (left current), PC1=IN11 (right current); UART banner. | 2025-12-18 | 2025-10-30 |
| 8 | Firmware v2: Dual-Motor Duty Bring-Up | <span style="color: green">Done</span> | M2 PWM/DIR (PA9/PB5) wired; duty sweep validated both channels; E-stop cut and GND common confirmed. | 2025-12-25 | 2025-11-03 |
| 9 | Firmware v2: Encoder Integration | <span style="color: green">Done</span> | Encoders online both wheels (TIM3 PA6/PA7 left, TIM2 PA0/PA1 right); UART RPM confirmed; direction corrected; TODO: add external 3.3 V pull-ups or software invert flag. | 2026-01-01 | 2025-11-16 |
| 10 | Firmware v2: Current Telemetry + Calibration | <span style="color: green">Done</span> | ADC1 scan IN8/IN11; zero-offset + scaling; filtered current stream; current reserved for logging/faults (not in loop). | 2026-01-08 | 2025-11-25 |
| 11 | Firmware v2: Control (Single-Loop PID) | <span style="color: green">Done</span> | TIM4 @100 Hz control loop; cmd_vel staleness timeout; speed PI + duty ramp; fault monitor (overcurrent/stall/encoder timeout/ADC stuck); hardware e-stop GPIO input + debounce wired into ControlState (latched fault); step/ramp plots + docs (docs/pid.md); gains/feedforward tuned. | 2026-01-15 | 2025-12-07 |
| 12 | Firmware v2: Cascaded Control + Comparison | <span style="color: goldenrod">In Progress</span> | Higher-accuracy current sensor integration is now active together with development of a safety/protection algorithm. Stay on single-loop speed control until the new sensing path is validated; then resume cascaded-control comparison work. | 2026-01-22 | TBD |
| 13 | Firmware v2: Differential Drive | <span style="color: green">Done</span> | Map (v, I%) -> wheel RPM; ramp/coordination added; saturation with curvature-preserving scaling; basic 5 s test sequence running. | 2026-01-29 | 2025-12-09 |
| 14 | Firmware v2: Proximity Sensors HW | <span style="color: goldenrod">In Progress</span> | Proximity sensor integration is now active: mounts, wiring, pull-ups/protection, pin-map updates, and bench power-budget checks are being worked through on the AMR. | 2026-02-05 | TBD |
| 15 | Firmware v2: Proximity Drivers | <span style="color: goldenrod">In Progress</span> | Driver bring-up and integration are now active: implement sampling scheduler, debouncing/filtering, fault detection, and telemetry exposure for the proximity sensors. | 2026-02-12 | TBD |
| 16 | Firmware v2: micro-ROS Bring-up | <span style="color: green">Done</span> | USART2 custom transport; `/cmd_vel` sub; `/amr/wheel_rpm_left`, `/amr/wheel_rpm_right`, `/amr/duty_cmd_left`, `/amr/duty_cmd_right`, `/amr/fault_mask`, `/amr/wheel_state`, `/amr/safety_state` pubs; `/amr/enable`, `/amr/estop`, `/amr/clear_fault` wired; fault clear verified; fault mask documented; legacy UART telemetry disabled. | 2026-02-19 | 2026-01-04 |
| 17 | Mechanical Assembly + Enclosure | <span style="color: green">Done</span> | Main AMR mechanical assembly completed, including SO-101 integration provisions. Follow-on hardware refinements now move into manipulation/perception work: wrist camera mount on SO-101, AMR depth sensor repositioning for better gripping, and any final cable routing/strain relief cleanup. | 2026-02-26 | 2026-04-17 |
| 18 | Dev PC Env & Tooling | <span style="color: green">Done</span> | Dockerized Foxy drivers + devpc image built (micro-ROS agent + YDLidar + RViz2/Gazebo); RViz2 config for `/scan` (best_effort QoS) working; ROS2 workspace build verified; Docker buildx workflow and Jetson builds validated; passwordless SSH to Jetson set up. | 2026-03-05 | 2026-01-18 |
| 19 | Jetson Nano ROS2/JetPack | <span style="color: green">Done</span> | Jetson container build complete; micro-ROS agent + teleop + YDLidar running; `/amr/*` and `/scan` verified; depth camera running with `/camera/*` and `/points` validated; RViz2 visualization from dev PC confirmed; DDS interface bound to Wi-Fi; depth stream stabilized with lower profiles. | 2026-03-12 | 2026-01-24 |
| 20 | Wireless PC<->Nano | <span style="color: green">Done</span> | Wi-Fi adapter online and Jetson reachable over home network; passwordless SSH working; DHCP reservation set; ping avg ~10 ms and iperf ~15 Mbps verified; NTP sync active. Optional VPN (WireGuard/Tailscale) remains a nice-to-have. | 2026-03-19 | 2026-01-20 |
| 21 | URDF Modeling (Base AMR) | <span style="color: green">Done</span> | Base URDF/Xacro created with chassis, wheels, and sensor frames (LiDAR + depth cam); LiDAR/camera split into `lidar.xacro` + `camera.xacro`; D455 camera plugin added; base_footprint restored; wheel offsets corrected; caster clearance tuned; Gazebo launch defaults to `obstacles.world`; ros2_control config added; URDF validated in sim. | 2026-03-26 | 2026-02-11 |
| 22 | Navigation & Mapping Bring-up | <span style="color: green">Done</span> | Mapping: slam_toolbox (2D LiDAR) running on real LiDAR; map save/load + posegraph serialization verified. Localization: AMCL on saved map working. Navigation: Nav2 bring-up working with RViz goals; costmaps wired to ros2_control topics. EKF fusion + odom improvements are tracked separately. | 2026-04-02 | 2026-02-11 |
| 23 | Navigation Validation & Safety | Planned | Regression routes; obstacle handling using LiDAR + depth; recovery behaviors; watchdogs/staleness; bag/latency logging; refine limits before adding arms; battery-powered navigation runs with Nav2 on dev PC/NUC (Jetson runs drivers only); validate IMU stability and drift. | 2026-04-09 | TBD |
| 24 | Task Executive & Sequencing (v1) | Planned | Choose BT/PlanSys2; define action interfaces (Navigate/Detect/Pick/Drop/Dock); implement mission executor; add retries/timeouts, status reporting, and logging. | 2026-04-16 | TBD |
| 25 | Mission Actions + Task Graph | Planned | Implement skill actions (stubs OK) and wire to Nav2/perception/manipulation; add world-state/blackboard; validate scripted sequences in sim. | 2026-04-23 | TBD |
| 26 | Voice Command MVP | Planned | Offline ASR (Vosk/Whisper) + intent parsing; confirmation prompts and safety gating; map intents to mission goals; log transcripts and outcomes. | 2026-04-30 | TBD |
| 27 | Mechanical Integration: Dual SO-101 Arms | <span style="color: green">Done</span> | Assembled dual SO-101 arms; verify reach/clearance; add power budget/fusing for arms; harness routing and strain relief; update CAD and pin/power map. | 2026-05-07 | 2026-02-08 |
| 28 | URDF/MoveIt for Base + Arms | Planned | Add SO-101 URDF/Xacro + collision meshes; integrate with base URDF/TF tree; generate MoveIt2 configs and limits; verify planning scene. | 2026-05-14 | TBD |
| 29 | Arm Control Bring-up (Bench) | Planned | Bring up arm drivers (ros2_control/trajectory action); joint state/trajectory streaming; homing/limits/soft-stops; basic Cartesian jogs. | 2026-05-21 | TBD |
| 30 | Calibration & Perception Baseline | <span style="color: goldenrod">In Progress</span> | Camera-mounting work is active on both SO-101 and the AMR-mounted camera/depth sensor. Follow with hand-eye and base-to-arm extrinsics, viewpoint validation for grasping/bagging, AprilTag validation, RGB-D grasp perception baseline (segmentation/keypoints), and the RGB-D+joints log pipeline. | 2026-05-28 | TBD |
| 31 | Teleop + Dataset Collection | Planned | Teleop/teaching tools (SpaceMouse/joystick); record synchronized video/joints/gripper for pick/place tasks; label successes/failures. | 2026-06-04 | TBD |
| 32 | Classical Grasp Pipeline | Planned | Perception -> grasp pose -> MoveIt planning/execution; guarded moves and retreat behaviors; bench metrics (success, cycle time, contact faults). | 2026-06-11 | TBD |
| 33 | Base+Arm Integration (Classical) | Planned | Navigate to pickup pose, align with RGB-D, run classical grasp, place at drop zone; recovery behaviors (regrasp/replan base pose) and watchdogs. | 2026-06-18 | TBD |
| 34 | VLA/VLM Model Trials (Sim) | Planned | Run state-of-the-art open VLA/VLM (OpenVLA/Octo/RT-class/diffusion) in sim with domain randomization; profile latency on Jetson/external GPU. | 2026-06-25 | TBD |
| 35 | VLA Guarded Hardware Replay | Planned | Deploy selected policies on hardware with action clamps/safety envelopes; compare to classical baseline; track success/intervention rate. | 2026-07-02 | TBD |
| 36 | End-to-End Autonomy Sprints | Planned | Full loop: navigate to goal -> pick -> place -> return; measure success, cycle time, collisions, latency; tighten limits/thresholds and logging. | 2026-07-09 | TBD |
| 37 | Field Bring-up (Supervised) | Planned | On-robot runs with manipulators: drive + pick/place in controlled environment; telemetry review; safety validation and recovery drills. | 2026-07-16 | TBD |
| 38 | PCB Concept & Requirements | Planned | Finalize end-state architecture (STM32 vs SOM, dual motor stage, rails, IO buses); measure/record real currents, noise, harness lengths; write electrical requirements. | 2026-07-23 | TBD |
| 39 | Carrier PCB (Dev Modules) | Planned | Design carrier/backplane for Nucleo + Cytron + external buck; connectors, power distribution, current sensing, ferrites/filters, ground planes; fab + bench bring-up. | 2026-07-30 | TBD |
| 40 | Custom Motor Driver / Production Prep | Planned | Begin custom H-bridge integration plan (DRV87xx + MOSFETs) and EMC/ESD prep; outline 4-layer stack, grounds, TVS/CMC, test points, panelization; plan pilot build. | 2026-08-06 | TBD |
| 41 | Voice I/O Expansion (On-Robot) | Planned | Add wake-word, noise suppression, and command queue; integrate mission status feedback; verify latency and reliability on Nano. | 2026-08-13 | TBD |
| 42 | Conversational TTS/Dialogue + Safety Guards | Planned | Add TTS responses, multi-turn clarification, and dialogue manager/LLM; explicit confirmations; test end-to-end voice -> nav/pick/place with guardrails. | 2026-08-20 | TBD |

> Canonical view rule: If the table and task board ever conflict, the table wins for schedule; task board wins for day-to-day details.

---

## ROS2 Control + Nav2 Roadmap (Phased)
- Phase 1 - Lock robot description
  - Finalize frames and geometry: base_footprint, base_link, left_wheel_joint, right_wheel_joint, lidar_link, camera_link.
  - Verify wheel positions, wheel radius, ground contact height, and total mass 12 kg.
  - Keep inertial and collision on all links; add gazebo tags when sim starts.
  - Goal: `ros2 launch amr_description view_urdf.launch.py` renders correctly.
- Phase 2 - Gazebo simulation
  - Add gazebo_ros2_control plugin block in URDF and transmissions for both wheel joints.
  - Create controller config YAML: joint_state_broadcaster and diff_drive_controller with wheel radius and separation.
  - Launch Gazebo, spawn robot, start controllers.
  - Goal: `/cmd_vel` moves robot in Gazebo and odom is produced.
- Phase 3 - ROS2 control hardware interface
  - Create package `amr_hardware` implementing hardware_interface::SystemInterface.
  - write publishes `/amr/wheel_cmd_left` and `/amr/wheel_cmd_right` in rad/s.
  - read consumes `/amr/wheel_state` for position and velocity.
  - Optional: handle `/amr/enable`, `/amr/estop`, `/amr/clear_fault`.
  - Goal: diff_drive_controller commands drive STM wheels.
- Phase 4 - Odometry and state estimation
  - Use diff_drive_controller odom first, then fuse IMU with robot_localization.
  - Ensure TF chain map -> odom -> base_link -> sensors.
  - Goal: stable odom and consistent TF.
- Phase 5 - Sensor bring-up
  - Lidar driver publishing /scan.
  - Depth camera topics and pointcloud.
  - Static transforms for lidar and camera relative to base.
  - Goal: stable sensor data in RViz with correct frames.
- Phase 6 - SLAM and Nav2 on dev PC
  - Run slam_toolbox to build map.
  - Configure Nav2 for footprint, inflation, costmaps.
  - Goal: teleop to map to Nav2 goal.
- Phase 7 - Split compute
  - Jetson runs drivers and micro-ROS agent.
  - Dev PC/NUC runs Nav2, slam_toolbox, and RViz.
  - Ensure network config and time sync are stable.

---

## Simulation & Navigation Breakdown - Foxy, Gazebo Classic
This section expands Weeks 18-23 with explicit simulation tasks using Gazebo Classic, ros2_control, slam_toolbox, and Nav2.

| Week | Simulation / Gazebo | ros2_control | slam_toolbox | Nav2 | Output |
|---:|---|---|---|---|---|
| 18 | Verify Gazebo Classic + RViz2 in devpc; create `amr_description`, `amr_gazebo`, `amr_bringup` packages; load a simple world | Stub diff_drive_controller config + controller_manager launch | N/A | N/A | Workspace scaffolding + Gazebo launches |
| 19 | Keep sim on dev PC; document Jetson limitations for sim | N/A | N/A | N/A | Jetson stays headless; sim remains on PC |
| 20 | Validate ROS graph over Wi-Fi between dev PC and Jetson; remote RViz2 from PC | N/A | N/A | N/A | Networked ROS2 verified (RViz on PC with Jetson sensors) |
| 21 | Base URDF/Xacro with LiDAR + depth frames and inertials added; Gazebo launch + spawn verified | ros2_control tags + controller YAML added; controller bring-up verified | N/A | N/A | Spawn + `/cmd_vel` drive verified |
| 22 | Add lidar plugin; verify `/scan`; add static transforms and base_link alignment | Tune wheel separation/radius, update controller params | Run slam_toolbox in sim; save map | Bring up Nav2 in sim; set params and run go-to-pose; test IMU fusion if simulated | Map saved; Nav2 go-to-pose works in sim |
| 23 | Run same pipeline on real sensors (or bag playback); compare sim vs real frames | Switch to real odom source; validate wheel_state->odom | Run slam_toolbox on real LiDAR; save a real map | Tune costmaps/footprint, recovery behaviors, and velocity limits; verify IMU fusion | Stable nav on real robot for 10-15 min |

---

## Detailed Tasks
Firmware tasks have been consolidated into the weekly tracker above. Use the Week-by-Week Plan as the single source of truth.

---

## Definitions of Done (per milestone)
- Dual-motor duty: Both channels drive 0-20% with correct direction and safe stop via E-stop
- Encoders: Stable RPM both wheels; correct direction; noise filtered
- Current telemetry: Calibrated A/LSB; filtered stream at 50-100 Hz
- Control (single-loop): <=10% overshoot, low SSE on step/ramp; plots archived
- Cascaded control: Inner current loop stable; outer speed loop SSE <= 5%, overshoot <= 10%; comparison plots and gains documented
- FreeRTOS (W13): Tasks meet deadlines under load; CPU < 70%; no missed watchdog
- Validation (W16): E-stop latency <= 50 ms; current-limit interaction stable under step loads
- SLAM demo (W19): Successful nav in mapped area for ~15 min without collision or watchdog resets
- Battery-powered Nav2: Run Nav2 on dev PC/NUC (Jetson runs drivers only); AMR drives to at least 3 waypoints without tether

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

## Repo Structure
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

## ROS2 Architecture
- Nodes
  - `mc_interface` (micro-ROS on STM32): subscribes `/cmd_vel`, `/amr/enable`, `/amr/estop`, `/amr/clear_fault`; publishes `/amr/wheel_rpm_left`, `/amr/wheel_rpm_right`, `/amr/duty_cmd_left`, `/amr/duty_cmd_right`, `/amr/fault_mask`, `/amr/wheel_state`, `/amr/safety_state`
  - `imu_driver` (Jetson): publishes `/imu` from the BNO080 over I2C
  - `odometry`: subscribes `/amr/wheel_state`; publishes wheel odom (input to robot_localization or directly to `/odom`)
  - `robot_localization` (optional): fuses wheel odom + IMU and publishes `/odom`, `/tf`
  - `safety_monitor`: subscribes `/amr/obstacles`, `/amr/estop`; publishes `/amr/safety_state`
  - `sensor_fusion`: fuses proximity/LiDAR/depth (and optionally IMU for health checks); publishes `/amr/obstacles`
  - `teleop` or higher-level commander: publishes `/cmd_vel` or `/amr/wheel_cmd`
  - `sim_bridge`: interfaces Gazebo topics with AMR topics
- Topics (current firmware)
  - `/cmd_vel` (geometry_msgs/Twist)
  - `/amr/enable` (std_msgs/Bool), `/amr/estop` (std_msgs/Bool), `/amr/clear_fault` (std_msgs/Empty)
  - `/amr/wheel_rpm_left` (std_msgs/Float32, RPM)
  - `/amr/wheel_rpm_right` (std_msgs/Float32, RPM)
  - `/amr/duty_cmd_left` (std_msgs/Float32, duty percent)
  - `/amr/duty_cmd_right` (std_msgs/Float32, duty percent)
  - `/amr/fault_mask` (std_msgs/Int32)
  - `/amr/wheel_state` (sensor_msgs/JointState)
  - `/amr/safety_state` (std_msgs/UInt32)
  - `/imu` (sensor_msgs/Imu)
- Topics (planned)
  - `/amr/wheel_cmd` (geometry_msgs/Twist or custom wheel velocities)
  - `/amr/obstacles` (sensor_msgs/Range[] or custom)
  - `/odom` (nav_msgs/Odometry), `/tf`, `/tf_static`

---

## Architecture Docs
- STM32 firmware architecture: `docs/STM_architecture.md`
- Jetson Nano runtime architecture: `docs/jetson_architecture.md`

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
[P1] Proximity sensor integration on AMR: mounting, wiring, driver bring-up, and telemetry
[P1] Integrate higher-accuracy current sensor and implement safety/protection algorithm
[P1] SO-101 wrist camera mount + cable routing for close-range grasp perception
[P1] AMR-mounted camera/depth-sensor placement update and transform refresh for better gripping/bagging coverage
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

2026-02-08 - Shifted to controlled A/B tuning + resume current sensing
Context: Early mapping worked but map noise and pose corrections highlighted slip/odom/control transients.
Decisions:
- Introduce firmware tuning profiles to isolate impact of launch guard and feedforward/static FF changes.
- Prioritize ACS758 current sensor validation next to enable current limiting and cascaded control.
Consequences:
- Faster iteration with fewer confounding variables.
- Clear path to industrial-style cascaded control once current sensing is trusted.

2026-02-11 - Standardized Nav2 bring-up (AMCL + navigation) on saved maps
Context: Saved maps are now available, but getting a repeatable bring-up required consistent QoS/TF behavior and correct topic wiring between Nav2 and ros2_control.
Decisions:
- Use map_server + AMCL for localization and Nav2 for navigation (slam_toolbox only for mapping sessions).
- Add wrapper launch + params to wire Nav2 `/cmd_vel` + `/odom` to diff_drive_controller topics.
- Update RViz config to subscribe to `/map` with transient-local QoS and include Nav2 goal tools.
Consequences:
- Faster, repeatable localization + navigation bring-up.
- Reduced “map not visible / missing map frame” confusion when RViz starts after map_server.

2026-04-17 - Shifted immediate manipulation focus to sensor placement for grasping
Context: SO-101 is now integrated, AMR mechanical assembly is complete, and an ACT policy has already demonstrated object pickup and placement into a bag.
Decisions:
- Mount a wrist/gripper camera on SO-101 for close-range manipulation views.
- Refine the AMR-mounted camera/depth-sensor placement to improve grasp and bagging visibility.
- Treat extrinsic recalibration as mandatory after both sensor changes before further ACT policy iteration.
Consequences:
- Better visual coverage near the gripper and target bag should improve grasp execution reliability.
- Sensor placement work becomes the next critical hardware task before deeper manipulation tuning.

2026-04-17 - Marked sensing and safety integration tracks as active work
Context: The immediate execution priorities are now proximity sensing, higher-accuracy current sensing with protection logic, and camera mounting on SO-101 plus the AMR.
Decisions:
- Mark proximity sensor hardware and driver integration as in progress.
- Mark higher-accuracy current sensor integration and safety algorithm development as in progress.
- Mark SO-101 camera mounting and AMR-mounted camera/depth-sensor refinement as in progress.
Consequences:
- The tracker now reflects current hands-on work instead of leaving these items in planned/backlog state.
- Near-term verification should focus on sensor bring-up quality, transform consistency, and safety behavior under real load.
```

---

## Change Log
- 2026-01-02: Added LiDAR/depth camera bring-up tasks; expanded navigation/mapping breakdown.
- 2026-01-02: Marked Weeks 17-18 in progress; noted enclosure prep completion and structural changes underway.
- 2026-01-30: URDF validated in sim; updated Week 21 to focus on sim-vs-real motion alignment; noted ros2_control Docker build reorder + realtime_tools header compatibility work.
- 2026-02-08: slam_toolbox mapping bring-up verified on real LiDAR; RViz config saved; created controlled firmware tuning profiles; encoder resolution/CPR validated on bench; began track width/wheel separation calibration; focus shifted back to current sensing to enable cascaded control.
- 2026-02-08: Assembled dual SO-101 arms (Week 27 marked done); mechanical integration ahead of schedule.
- 2026-02-11: AMCL + Nav2 navigation validated on saved map; added Nav2 params + bring-up launch (topic remaps); RViz config updated (Nav2 panel, `/map` QoS); tuned local costmap size for navigation.
- 2026-04-17: Marked Week 17 mechanical assembly complete; noted SO-101 integration, successful ACT pick-and-place-into-bag demo, and next focus on wrist camera mounting + AMR depth sensor repositioning.
- 2026-04-17: Marked proximity sensor integration, higher-accuracy current sensor + safety algorithm work, and camera mounting on SO-101/AMR as in progress in the canonical tracker.
- 2026-01-01: Added functional hardware e-stop input with debounce + fault latch; tracker updated to reflect current clear-path gap.
- 2025-12-22: Integrated mechanical CAD tasks into Week 17; renamed STM architecture doc to `docs/STM_architecture.md`; removed merged task/architecture files.
- 2025-12-22: Consolidated firmware tasks into weekly tracker; merged STM32 architecture docs; added Jetson Nano architecture doc.
- 2025-12-22: Added Week 17 mechanical assembly + enclosure tasks; shifted schedule by one week.
- 2025-12-22: micro-ROS bring-up on STM32 (USART2 transport, `/cmd_vel` sub, RPM + fault mask pubs); updated ROS topic list.
- 2025-10-28: Marked Week 2 ADC as skipped; set Week 8 next focus to E-stop feature; added Final Drivetrain Migration step; noted flexible timeline and PCB as optional.
- 2025-11-XX: Started AMR CAD assembly with dual SO101 arm manipulators mounted; initial rough layout committed to repo.
- 2025-11-XX: Added main power switch at pack output ahead of fuse/E-Stop; updated hardware block diagram accordingly.
- 2025-11-XX: Added DSN-DVM/DUM-368 battery voltage display (fed after main switch) to specs and wiring docs.
- 2025-11-XX: Selected 4x HC-SR04 ultrasonic proximity sensors (trig/echo to STM32); updated specs, wiring, and diagram.
- 2025-11-XX: Power split: XH-M401 (XL4016 class) for Jetson/hub 5 V rail; LM2596 for logic/proximity 5 V rail.
- 2025-11-XX: Cascaded current loop deferred; continue single-loop speed PI until higher-accuracy current sensor is added.

---

## Manipulator + AI Roadmap (SO-101 arms with VLA/VLM/LLM stack)

Goal: two open-source SO-101 arms mounted on the AMR, capable of autonomous pick/place of small objects. Control runs through ROS2/micro-ROS, with state-of-the-art vision-language-action (VLA), vision-language (VLM), and LLM-based planners on the Jetson or a paired edge GPU when needed.

### Phase A: Mechanical + Electrical Bring-up
- Mount both SO-101 arms on the chassis (verify payload, reach, FOV clearances). Add strain relief and service loops for cables.
- Power: size 5 V/12 V rails or separate buck for the arms; confirm inrush/peak current; fuse per arm. Tie grounds at star point.
- IO: decide arm drivers (existing SO-101 controller or custom): interface via USB/RS485/CAN; level-shift if needed; map pins/connectors.
- Safety: add arm E-stop integration (shared with base) and per-arm enable; confirm brakes/hold torque expectations.

### Phase B: Kinematics, URDF, and ROS2 Control
- Create SO-101 URDF/Xacro with collision/visual meshes; add to the AMR URDF with correct frames and inertias.
- Calibrate DH/offsets and tool frames; define grasp frame and camera frames (RealSense + LiDAR + any wrist cam if added).
- Bring up control stack (MoveIt 2 or ros2_control drivers): joint state publisher, trajectory action server, and gripper control.
- Run bench tests: joint jogging, homing, limits, soft-stops, and basic Cartesian moves.

### Phase C: Perception and Calibration
- Extrinsics: hand-eye calibration (if wrist cam), base-to-arm, and camera-to-arm transforms; store in YAML and validate with AprilTags.
- Grasp perception: start with classical + learned hybrids (AprilTags/ArUco for fixtures, then segmentation/keypoint nets on RGB-D).
- Build data/logging pipeline for RGB-D + joint states + gripper status + base pose; include bagging scripts and dataset versioning.

### Phase D: Teleop and Dataset Collection
- Add teleop/kinesthetic teaching tools: SpaceMouse/VR controller/joystick + MoveIt task recording; log synchronized video and joint trajectories.
- Collect canonical tasks: pick/place from bins/table, handoff to base, simple sorting. Label successes/failures; track intervention rate.
- Generate synthetic data in sim (Gazebo) with domain randomization for lighting/textures/poses to augment the real dataset.

### Phase E: Classical Baselines
- Baseline pipeline: perception -> grasp pose -> Motion planning (MoveIt RRT*/OMPL) -> execution with compliance/velocity caps.
- Add guarded moves (force/torque thresholds if available) and retreat behaviors; log metrics (success, time-to-grasp, contact faults).

### Phase F: VLA/VLM/LLM Integration (state-of-the-art models)
- VLA policies for manipulation: prototype with open models (e.g., OpenVLA-style policies, Octo/RT-1/RT-2 class, diffusion policies) conditioned on RGB-D + text; start in sim, then replay on hardware with action clamps and safety envelopes.
- VLMs for perception: try segment-anything/grounding-style models for object proposals; fuse with depth for grasp pose proposals.
- LLM for task planning: route high-level instructions to sequences (navigate -> detect -> pick -> place). Keep execution guarded by deterministic controllers; enforce staleness timeouts and safety checks.
- Jetson constraints: profile latency; offload heavy models to external GPU if needed; fall back to lighter distilled variants on Nano.

### Phase G: Policy Training and Evaluation
- Training loop: dataset curation -> supervised behavior cloning for pick/place primitives -> fine-tune with RL or offline RL where feasible.
- Safety guards: action filters (joint limits, velocity caps, collision checks), watchdogs, and automatic stop on perception dropouts.
- Metrics: success rate per object/scene, grasp quality, cycle time, collision/near-miss count, latency budget, sim-to-real gap.
- A/B compare: classical pipeline vs VLA policy vs hybrids (VLA proposes grasp + classical planner executes).

### Phase H: Integrated Autonomy with Base
- Task graph: plan base pose for manipulation (nav goal), align using LiDAR/depth, then run arm sequence; recover on failure (regrasp, replan base pose).
- Multi-modal sensing: fuse LiDAR/depth/proximity for base; use RGB-D for arms; maintain transforms so grasps align with base frame.
- Field tests: constrained lab runs -> supervised floor runs -> limited unsupervised with watchdog; track intervention rate and MTBF.

### Deliverables per phase
- CAD mounts, updated URDF, MoveIt configs, calibration files.
- ROS2 packages: arm drivers, perception nodes, task orchestrator, policy runner with safety filters.
- Datasets and training scripts; evaluation reports with metrics and plots.
- Safety checklist: E-stop integration, speed/force limits, tested recovery behaviors.

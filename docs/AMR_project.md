# Kartik's AMR Project Tracker (42 Weeks)
**File:** `AMR_project.md`  
**Owner:** Kartik Mehta  
**Last Updated:** 2026-04-28  
**Scope:** STM32 low-level control, Jetson Nano high-level compute, motor drivers, current sensing (ACS758 x2), FreeRTOS, ROS2 + Gazebo, SLAM & Navigation; eventual goal is a fully autonomous AMR with dual SO-101 manipulators that can pick/place small objects using state-of-the-art VLA/VLM/LLM-based policies.

---

## Status Summary
- Overall: SLAM + localization + navigation pipeline is working on real hardware (slam_toolbox map -> saved map -> AMCL -> Nav2 goals); one-shot Jetson+dev-PC bring-up scripts now include STM reset through ST-LINK/OpenOCD; the mission layer now runs as a persistent dev-PC-side runtime on top of Nav2 with typed status/state and integrated tmux panes; SO-101 integration and an initial ACT manipulation demo are now validated; low-level control/odometry calibration is still in active tuning.
- Progress: 20/42 weeks complete (~48%) (plan is now being executed iteratively vs. strictly week-by-week).
- Recent:
  - Reflashed `STM_Firmware_AMR_v2`, restored STM32 micro-ROS connectivity, and revalidated live `/amr/*` topics on the Jetson bring-up path.
  - Re-checked wheel encoders on live `/amr/wheel_state`, corrected left/right motor-channel wiring, and fixed wheel direction polarity so individual left/right wheel commands now actuate the intended side.
  - Added ROS current topics (`/amr/current_left_ma`, `/amr/current_right_ma`) plus raw ADC/zero topics for bench bring-up and calibration of the replacement current sensors.
  - Completed current-sensor integration bring-up: current sign/polarity was corrected, idle offset reduced to a small near-zero bias, load current now rises in the expected direction on both sides, and current settles back near zero when the wheels stop.
  - Restored the overcurrent threshold to `1500 mA` after calibration and confirmed the firmware-side current path is now usable for protection testing.
  - Added a one-command Jetson bench monitor workflow for wheel state, duty, current, safety state, and fault tracking during bring-up.
  - slam_toolbox bring-up working with real LiDAR (`/scan` -> `/map`) and RViz config saved.
  - Saved a real map and validated map_server + AMCL localization; Nav2 navigation (planner/controller) working with RViz goal tool.
  - Added a Nav2 wrapper launch + params to remap `/cmd_vel` + `/odom` to ros2_control diff-drive topics and to launch RViz reliably inside Docker.
  - Added one-shot dev-PC workflows for SLAM and navigation that first bring up the Jetson hardware stack and then reset the STM over ST-LINK using `openocd` on the Jetson.
  - Validated noninteractive STM reset from the Jetson with `sudo -n openocd -s /usr/share/openocd/scripts -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg -c "init; reset run; shutdown"`.
  - Fixed RViz map display QoS (Transient Local + Reliable) and added Nav2 UI tools/panel to `amr.rviz`.
  - Marked Week 21 (URDF base) and Week 22 (mapping/localization/navigation bring-up) complete; EKF fusion is tracked as a separate improvement.
  - Resolved TF bring-up issues by ensuring ros2_control/controller_manager + `/joint_states` are alive before SLAM.
  - LiDAR alignment parameterized in URDF (yaw adjust) and rebuilt into runtime container workflow.
  - Encoder pipeline validated on bench (manual rotations), moved to higher-resolution timer mode (TI12), and began closing the loop on wheel kinematics (track width / wheel separation) using 360-degree tests.
  - Added firmware tuning profiles for controlled A/B testing (launch guard, feedforward/static FF, etc.) to avoid “many knobs at once”.
  - Promoted `amr_missions` into a persistent mission runtime with `mission_server`, typed `MissionStatus`, `/amr_missions/state`, and a command topic; the one-shot navigation launcher now includes mission-server, mission-status, and mission-command panes.
  - Revalidated named-place navigation and patrol flows on real hardware with the persistent mission runtime; current saved places are `home`, `door`, `kitchen`, and `hall`.
  - Integrated SO-101 into the active project stack and completed the main AMR mechanical assembly.
  - Ran an ACT policy that successfully picked up an object and placed it into a bag.
- micro-ROS: STM32 bring-up on USART2 with full AMR topic set live (wheel_state + duty topics visible); UART telemetry disabled to avoid contention.
- Safety: Hardware e-stop GPIO integrated with debounce and fault latch.
- Current Focus (next sprint):
  - Add Jetson-side EKF state estimation (`robot_localization`) using wheel odom plus camera IMU, then retest SLAM/AMCL/Nav2 on filtered odom.
  - Extend the persistent mission runtime with named routes, stronger supervision, and tighter integration with the rest of the autonomy stack.
  - Add safety/health supervision for LiDAR freshness, localization validity, TF/odom staleness, STM comm health, and mission abort/stop behavior.
  - Add a first voice I/O scaffold: speaker on AMR for TTS/status and laptop-mic command intake for a small set of mission commands.
  - Keep proximity and manipulation/perception integration moving only as needed to support the above, but defer heavier perception-assisted autonomy until the Orin NX arrives.
- Control Architecture Direction:
  - Yes, moving toward cascaded control makes sense: inner current/torque limiting (or current loop if feasible) + outer speed loop is the standard industrial structure and will reduce slip/launch transients once current sensing is reliable.
- Next Focus:
  - Close the loop on traction/launch transients (feedforward + ramp + slip) using current + filtered odom; validate longer battery-powered Nav2 runs with EKF enabled.
  - Promote place-based navigation into a robust mission layer that no longer depends on shell-only workflows.
  - Once the Jetson Orin NX arrives, move on-robot perception, semantic autonomy, and manipulator runtime there.
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
| 12 | Firmware v2: Cascaded Control + Comparison | <span style="color: goldenrod">In Progress</span> | Higher-accuracy current-sensor integration is now functionally complete for bench bring-up and protection work: `STM_Firmware_AMR_v2` was reflashed, wheel-state and individual wheel commands were revalidated, ROS current + ADC/zero topics were added, current polarity/sign were corrected, and the OC threshold was returned to `1500 mA`. Stay on single-loop speed control for now; next step is Jetson-side odometry/slip work, then resume cascaded-control comparison once traction behavior is better characterized. | 2026-01-22 | TBD |
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
| 23 | EKF State Estimation & Nav Validation | Planned | Integrate `robot_localization`; fuse wheel odom + camera IMU; publish filtered odom/TF; retest SLAM, AMCL, and Nav2 on the improved state estimate; run longer battery-powered navigation routes and measure drift. | 2026-04-09 | TBD |
| 24 | Mission Runtime & Sequencing (v1) | <span style="color: green">Done</span> | Promoted `amr_missions` from CLI-only use into a persistent `mission_server` with typed `MissionStatus`, a `/amr_missions/state` service, topic-command support, request validation, retries/timeouts/return-home handling, and direct integration into the one-shot navigation tmux workflow. | 2026-04-16 | 2026-04-28 |
| 25 | Safety Supervision & Health Monitoring | Planned | Add watchdogs for LiDAR freshness, localization validity, TF/odom staleness, STM comm health, and Nav2 stuck/timeout detection; add safe-stop / mission-abort behavior and health summary topics. | 2026-04-23 | TBD |
| 26 | Voice Command MVP | Planned | Add speaker output plus a first voice interface using laptop mic input; offline ASR (Vosk/Whisper) + intent parsing + confirmation prompts; map a small command set (`go home`, `go kitchen`, `stop`, `status`) into mission goals and log transcripts/outcomes. | 2026-04-30 | TBD |
| 27 | Mechanical Integration: Dual SO-101 Arms | <span style="color: green">Done</span> | Assembled dual SO-101 arms; verified reach/clearance; added power budget/fusing direction for arms; harness routing and strain relief; updated CAD and pin/power map. | 2026-05-07 | 2026-02-08 |
| 28 | URDF/MoveIt for Base + Arms | Planned | Add SO-101 URDF/Xacro + collision meshes; integrate with base URDF/TF tree; calibrate tool/grasp frames and camera frames; generate MoveIt2 configs and limits; verify planning scene. This is still mostly independent of the compute upgrade. | 2026-05-14 | TBD |
| 29 | Arm Control Bring-up (Bench) | Planned | Bring up arm drivers (ros2_control/trajectory action or equivalent IO path); joint state/trajectory streaming; gripper control; homing/limits/soft-stops; basic Cartesian jogs. Keep this bench-focused before shifting runtime onto the Orin NX. | 2026-05-21 | TBD |
| 30 | Calibration & Perception Baseline | <span style="color: goldenrod">In Progress</span> | Camera-mounting work is active on both SO-101 and the AMR-mounted camera/depth sensor. Follow with hand-eye and base-to-arm extrinsics, viewpoint validation for grasping/bagging, AprilTag validation, RGB-D grasp perception baseline (segmentation/keypoints or grounding-style proposals), and the RGB-D+joints log pipeline. Heavier on-robot perception compute is expected to shift to the Jetson Orin NX once available. | 2026-05-28 | TBD |
| 31 | Teleop + Dataset Collection | Planned | Add teleop/teaching tools (SpaceMouse/joystick/recording workflow); record synchronized video, joints, gripper state, and base pose for pick/place tasks; label successes/failures; generate sim/domain-randomized data where useful. Prepare datasets and logs now so they are ready for Orin-based perception/runtime experiments. | 2026-06-04 | TBD |
| 32 | Classical Grasp Pipeline | Planned | Perception -> grasp pose -> MoveIt planning/execution; guarded moves, retreat behaviors, and compliance/velocity caps; bench metrics (success, cycle time, contact faults). Keep this as the pre-Orin baseline against which later learned/perception-heavy approaches are compared. | 2026-06-11 | TBD |
| 33 | Base+Arm Integration (Classical) | Planned | Navigate to pickup pose, align with RGB-D, run classical grasp, place at drop zone; add recovery behaviors (regrasp/replan base pose) and watchdogs; maintain consistent base/arm/camera transforms. Use this as the last major base+arm milestone before moving heavier runtime onto the Orin NX. | 2026-06-18 | TBD |
| 34 | Orin NX Bring-up + Perception Runtime Migration | Planned | Bring up the new Jetson Orin NX, reproduce the ROS2/docker stack, move on-robot perception workloads there, validate cameras/depth/IMU throughput, and confirm manipulator-control processes can run on-robot with acceptable latency. | 2026-06-25 | TBD |
| 35 | VLA/VLM Model Trials (Sim + Orin Profiling) | Planned | Run state-of-the-art open VLA/VLM (OpenVLA/Octo/RT-class/diffusion) in sim with domain randomization; evaluate VLM-driven object proposals with depth fusion; profile latency on the Orin NX and any external GPU fallback; choose an initial perception/autonomy runtime stack. | 2026-07-02 | TBD |
| 36 | VLA Guarded Hardware Replay | Planned | Deploy selected policies on hardware with action clamps/safety envelopes; compare to the classical baseline; track success/intervention rate; verify Orin-side perception/manipulation runtime stability; keep execution guarded by deterministic controllers. | 2026-07-09 | TBD |
| 37 | End-to-End Autonomy Sprints | Planned | Full loop: navigate to goal -> detect -> pick -> place -> return using the mission runtime + Orin-based perception/manipulation stack; measure success, cycle time, collisions, latency, and sim-to-real gap; tighten limits/thresholds and logging. | 2026-07-16 | TBD |
| 38 | PCB Concept & Requirements | Planned | Finalize end-state architecture (STM32 vs SOM, dual motor stage, rails, IO buses); measure/record real currents, noise, harness lengths; write electrical requirements. | 2026-07-23 | TBD |
| 39 | Carrier PCB (Dev Modules) | Planned | Design carrier/backplane for Nucleo + Cytron + external buck; connectors, power distribution, current sensing, ferrites/filters, ground planes; fab + bench bring-up. | 2026-07-30 | TBD |
| 40 | Custom Motor Driver / Production Prep | Planned | Begin custom H-bridge integration plan (DRV87xx + MOSFETs) and EMC/ESD prep; outline 4-layer stack, grounds, TVS/CMC, test points, panelization; plan pilot build. | 2026-08-06 | TBD |
| 41 | Voice I/O Expansion (On-Robot) | Planned | Move voice input/output fully onto the robot; add wake-word, noise suppression, and command queue; integrate mission status feedback; verify latency and reliability on the Jetson Orin NX. | 2026-08-13 | TBD |
| 42 | Conversational TTS/Dialogue + Safety Guards | Planned | Add TTS responses, multi-turn clarification, and dialogue manager/LLM; explicit confirmations; test end-to-end voice -> nav/pick/place with guardrails on the Orin-based stack. | 2026-08-20 | TBD |

> Canonical view rule: If the table and task board ever conflict, the table wins for schedule; task board wins for day-to-day details.


## PCB Migration Milestones (dev boards -> integrated AMR control PCB)
- Conceptual architecture (target end-state): Define final board contents (STM32/bare MCU or SOM, dual motor stage, buck rails 12->5->3.3 V, encoder conditioning, current sense, battery protection, all IO connectors, CAN/UART/RS485/I2C). This is the "Cytron + Nucleo + buck + encoder + UART + IO" rolled into one.
- Phase A - Measure on dev boards: With Nucleo + Cytron + XL4016, capture real currents (continuous/peak), encoder voltage tolerance/noise, UART/CAN bandwidth, EMI/ground noise patterns, ADC resolution needs, harness lengths and connector types. This produces the electrical requirements doc.
- Phase B - Consolidation carrier PCB: Keep Nucleo + Cytron + external buck, but design a carrier/backplane that handles connectors, power distribution, current sensing, ferrite/filter caps, and ground planes to organize wiring and validate signal/power integrity.
- Phase C - Integrate motor driver: Drop Cytron; design your own dual H-bridge with gate driver (e.g., DRV87xx) + MOSFETs + shunts/Hall sensors. Validate thermals, switching, protection (TVS/fuse/reverse), and EMC.
- Phase D - Integrate MCU: Replace Nucleo with bare STM32F401 (LQFP-64): crystal, boot config, SWD header, decoupling, ESD/TVS, brownout protection. Port firmware; bring-up clocks, debug, boot, and motor control.
- Phase E - Production/EMC-ready: Move to 4-layer, split/stitched grounds, Kelvin sensing, star grounds, TVS + common-mode chokes, reverse/fuse protection, panelization notes, silks/labels, test points. Run pre-scan EMC/ESD and pilot build (5-10 boards) with harnesses and acceptance tests.
---


## Architecture Docs
- ROS stack diagrams: `docs/ros_stack_diagrams.md`
- STM32 firmware architecture: `docs/STM_architecture.md`
- Jetson Nano runtime architecture: `docs/jetson_architecture.md`


---

## Project Log
- 2026-04-27: Updated the canonical roadmap to split near-term work and post-Orin work more cleanly. Short-term milestones are now EKF state estimation, persistent mission runtime, safety supervision, and voice I/O scaffolding. Perception-heavy autonomy and manipulator runtime migration are explicitly deferred to the Jetson Orin NX phase.
- 2026-04-27: Added one-shot dev-PC workflows for SLAM and navigation that first bring up the Jetson hardware stack, then reset the STM over ST-LINK via `openocd`, then launch the desktop-side session. Validated noninteractive reset with `sudo -n openocd -s /usr/share/openocd/scripts -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg -c "init; reset run; shutdown"`.
- 2026-04-27: Added `amr_missions`, a first mission-layer ROS 2 package with YAML-defined named places and a `mission_cli` supporting `list`, `go_to <place>`, and `patrol ... --return-home ...`. Validated named-place navigation on real hardware for `kitchen` and `hall`; current saved places are `home`, `door`, `kitchen`, and `hall`.
- 2026-04-28: Promoted `amr_missions` into a persistent dev-PC-side mission runtime: added `mission_server`, typed `MissionStatus` publishing on `/amr_missions/status`, a `/amr_missions/state` service, topic-command control, stricter request validation, and mission-server/status/command panes in the one-shot navigation tmux workflow. Patrol was revalidated before merge.
- 2026-04-22: Closed the current-sensor integration bring-up loop for bench use: added raw ADC/zero ROS topics and a one-command Jetson monitor, corrected current polarity/sign, brought idle offsets down near zero on both sides, restored the overcurrent threshold to `1500 mA`, and shifted next-focus planning toward Jetson-side odometry fusion/slip detection.
- 2026-04-19: Reflashed `STM_Firmware_AMR_v2`; recovered micro-ROS bring-up; revalidated encoder feedback and individual left/right wheel control; added ROS current topics; corrected wheel direction polarity; flipped right current polarity and temporarily raised the OC threshold for current-sensor calibration bring-up; noted remaining idle-zero drift on the replacement current sensor.
- 2026-04-17: Marked Week 17 mechanical assembly complete; noted SO-101 integration, successful ACT pick-and-place-into-bag demo, and next focus on wrist camera mounting + AMR depth sensor repositioning.
- 2026-04-17: Shifted immediate manipulation focus to sensor placement for grasping. Decisions: mount a wrist/gripper camera on SO-101, refine the AMR-mounted camera/depth-sensor placement, and treat extrinsic recalibration as mandatory before further ACT policy iteration.
- 2026-04-17: Marked proximity sensor integration, higher-accuracy current sensor + safety algorithm work, and camera mounting on SO-101/AMR as active tracks in the canonical plan.
- 2026-02-11: Standardized Nav2 bring-up on saved maps. Decisions: use map_server + AMCL for localization, use Nav2 for navigation, add wrapper launch/params to wire `/cmd_vel` and `/odom` to ros2_control topics, and update RViz config to use transient-local `/map` QoS with Nav2 tools.
- 2026-02-08: slam_toolbox mapping bring-up verified on real LiDAR; RViz config saved; created controlled firmware tuning profiles; encoder resolution/CPR validated on bench; began track width/wheel separation calibration; focus shifted back to current sensing to enable cascaded control.
- 2026-02-08: Assembled dual SO-101 arms (Week 27 marked done); mechanical integration ahead of schedule.
- 2026-02-08: Shifted to controlled A/B tuning + current-sensor-first control work. Decisions: isolate firmware tuning changes via profiles and prioritize ACS758 validation to unlock current limiting and later cascaded control.
- 2026-01-30: URDF validated in sim; updated Week 21 to focus on sim-vs-real motion alignment; noted ros2_control Docker build reorder + realtime_tools header compatibility work.
- 2026-01-02: Added LiDAR/depth camera bring-up tasks; expanded navigation/mapping breakdown; marked Weeks 17-18 in progress and noted enclosure prep completion and structural changes underway.
- 2026-01-01: Added functional hardware e-stop input with debounce + fault latch; tracker updated to reflect the new clear-path safety baseline.
- 2025-12-22: Integrated mechanical CAD tasks into Week 17; renamed STM architecture doc to `docs/STM_architecture.md`; removed merged task/architecture files.
- 2025-12-22: Consolidated firmware tasks into the weekly tracker; merged STM32 architecture docs; added Jetson Nano architecture doc.
- 2025-12-22: Added Week 17 mechanical assembly + enclosure tasks; shifted schedule by one week.
- 2025-12-22: micro-ROS bring-up on STM32 (USART2 transport, `/cmd_vel` sub, RPM + fault mask pubs); updated ROS topic list.
- 2025-11-XX: Started AMR CAD assembly with dual SO-101 arm manipulators mounted; initial rough layout committed to repo.
- 2025-11-XX: Added main power switch at pack output ahead of fuse/E-stop; updated hardware block diagram accordingly.
- 2025-11-XX: Added DSN-DVM/DUM-368 battery voltage display (fed after main switch) to specs and wiring docs.
- 2025-11-XX: Selected 4x HC-SR04 ultrasonic proximity sensors (trig/echo to STM32); updated specs, wiring, and diagram.
- 2025-11-XX: Power split defined: XH-M401 (XL4016 class) for Jetson/hub 5 V rail; LM2596 for logic/proximity 5 V rail.
- 2025-11-XX: Cascaded current loop deferred; continue single-loop speed PI until higher-accuracy current sensing is ready.
- 2025-10-28: Marked Week 2 ADC as skipped; set Week 8 next focus to E-stop feature; added Final Drivetrain Migration step; noted flexible timeline and PCB as optional.

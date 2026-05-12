# Kartik's AMR Project Tracker (48 Weeks)
**File:** `AMR_project.md`  
**Owner:** Kartik Mehta  
**Last Updated:** 2026-05-12
**Scope:** STM32 low-level control, Jetson Nano high-level compute, motor drivers, current sensing (ACS758 x2), FreeRTOS, ROS2 + Gazebo, SLAM & Navigation; eventual goal is a fully autonomous AMR with dual SO-101 manipulators that can pick/place small objects using state-of-the-art VLA/VLM/LLM-based policies.

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
| 11 | Firmware v2: Control (Single-Loop PID) | <span style="color: green">Done</span> | TIM4 @100 Hz control loop; cmd_vel staleness timeout; speed PI + duty ramp; fault monitor (overcurrent/stall/encoder timeout/ADC stuck); hardware e-stop GPIO input + debounce wired into ControlState (latched fault); step/ramp plots captured during bring-up; gains/feedforward tuned. | 2026-01-15 | 2025-12-07 |
| 12 | Firmware v2: Cascaded Control + Comparison | <span style="color: green">Done</span> | Cascaded-current-control investigation is complete for this AMR phase. Higher-accuracy current-sensor integration was brought up for protection/diagnostics, ROS current + ADC/zero topics were added, current polarity/sign were corrected, and the OC threshold was returned to `1500 mA`. Decision: keep production motion on the validated single-loop speed controller for now; use current sensing for protection/logging and revisit true cascaded current control only if traction or torque-control requirements justify it. | 2026-01-22 | 2026-05-03 |
| 13 | Firmware v2: Differential Drive | <span style="color: green">Done</span> | Map (v, I%) -> wheel RPM; ramp/coordination added; saturation with curvature-preserving scaling; basic 5 s test sequence running. | 2026-01-29 | 2025-12-09 |
| 14 | Firmware v2: Proximity Sensors HW | Planned | Original proximity hardware slot. Execution is now rolled into the near-term AMR base completion work: mounts, wiring, pull-ups/protection, pin-map updates, power-budget checks, and integration with STM telemetry. | 2026-02-05 | TBD |
| 15 | Firmware v2: Proximity Drivers | Planned | Original proximity driver slot. Execution is now rolled into the near-term AMR base completion work: sampling scheduler, debouncing/filtering, crosstalk handling, fault detection, and ROS telemetry exposure. | 2026-02-12 | TBD |
| 16 | Firmware v2: micro-ROS Bring-up | <span style="color: green">Done</span> | USART2 custom transport; `/amr_stm/wheel_cmd_left` and `/amr_stm/wheel_cmd_right` subs; `/amr_stm/wheel_state`, duty, current, ADC/zero, fault, safety, and `/amr_stm/ros_diag` pubs; `/amr_stm/enable`, `/amr_stm/estop`, `/amr_stm/clear_fault` wired; fault clear verified; fault mask documented; legacy UART telemetry disabled. | 2026-02-19 | 2026-01-04 |
| 17 | Mechanical Assembly + Enclosure | <span style="color: green">Done</span> | Main AMR mechanical assembly completed, including SO-101 integration provisions. Follow-on hardware refinements now move into manipulation/perception work: wrist camera mount on SO-101, AMR depth sensor repositioning for better gripping, and any final cable routing/strain relief cleanup. | 2026-02-26 | 2026-04-17 |
| 18 | Dev PC Env & Tooling | <span style="color: green">Done</span> | Dockerized Foxy drivers + devpc image built (micro-ROS agent + YDLidar + RViz2/Gazebo); RViz2 config for `/scan` (best_effort QoS) working; ROS2 workspace build verified; Docker buildx workflow and Jetson builds validated; passwordless SSH to Jetson set up. | 2026-03-05 | 2026-01-18 |
| 19 | Jetson Nano ROS2/JetPack | <span style="color: green">Done</span> | Jetson container build complete; micro-ROS agent + teleop + YDLidar running; `/amr_stm/*` and `/scan` verified; depth camera running with `/camera/*` and `/points` validated; RViz2 visualization from dev PC confirmed; DDS interface bound to Wi-Fi; depth stream stabilized with lower profiles. | 2026-03-12 | 2026-01-24 |
| 20 | Wireless PC<->Nano | <span style="color: green">Done</span> | Wi-Fi adapter online and Jetson reachable over home network; passwordless SSH working; DHCP reservation set; ping avg ~10 ms and iperf ~15 Mbps verified; NTP sync active. Optional VPN (WireGuard/Tailscale) remains a nice-to-have. | 2026-03-19 | 2026-01-20 |
| 21 | URDF Modeling (Base AMR) | <span style="color: green">Done</span> | Base URDF/Xacro created with chassis, wheels, and sensor frames (LiDAR + depth cam); LiDAR/camera split into `lidar.xacro` + `camera.xacro`; D455 camera plugin added; base_footprint restored; wheel offsets corrected; caster clearance tuned; Gazebo launch defaults to `obstacles.world`; ros2_control config added; URDF validated in sim. | 2026-03-26 | 2026-02-11 |
| 22 | Navigation & Mapping Bring-up | <span style="color: green">Done</span> | Mapping: slam_toolbox (2D LiDAR) running on real LiDAR; map save/load + posegraph serialization verified. Localization: AMCL on saved map working. Navigation: Nav2 bring-up working with RViz goals; costmaps wired to ros2_control topics. EKF fusion + odom improvements are tracked separately. | 2026-04-02 | 2026-02-11 |
| 23 | IMU + EKF State Estimation & Nav Validation | Planned | Decide and validate the reliable IMU source first. Near-term hardware plan is external BNO080 or equivalent on planned STM I2C `PB8/PB9`; alternatives remain RealSense D455 IMU or another Orin-compatible IMU if STM bring-up is not reliable. Validate `/imu` topic rate, timestamps, frame ID, orientation/gyro stability, covariance, and mounting before enabling `robot_localization`. Then fuse wheel odom + IMU, define filtered-odom TF ownership, retest SLAM, AMCL, and Nav2, and include follow-up for motion-only AMCL/localization jumps seen during 2026-05-03 mission validation. | 2026-04-09 | TBD |
| 24 | Mission Runtime & Sequencing (v1) | <span style="color: green">Done</span> | Promoted `amr_missions` from CLI-only use into a persistent `mission_server` with typed `MissionStatus`, a `/amr_missions/state` service, topic-command support, request validation, retries/timeouts/return-home handling, and direct integration into the one-shot navigation tmux workflow. | 2026-04-16 | 2026-04-28 |
| 25 | Safety Supervision & Health Monitoring | <span style="color: green">Done</span> | Implemented and validated the first production safety-supervision layer: STM comm/fault monitoring, safety intervention state, mission/Nav2 cancel path, guarded zero-velocity + STM disable flow, supervisor reset service, and repeatable operator recovery helper. Real motor-driver power-cut tests confirmed safe stop, blocked reset while STM faults were active, fault clear after power restore, manual STM re-enable, and successful mission continuation. Follow-up safety expansions can be tracked separately. | 2026-04-23 | 2026-05-03 |
| 26 | Voice + Operator Command MVP | <span style="color: goldenrod">In Progress</span> | MCP-oriented voice/operator path is active: `hey jarvis` wake gating, Vosk command ASR and Faster Whisper push-to-talk support, Piper TTS through speaker MCP, conversation MCP planning, Qwen-backed local intent routing, pending confirmation handling, and read-only state-inspection tool execution for status/diagnostics. Motion remains gated through mission-control MCP readiness and explicit supervised confirmation. | 2026-04-30 | TBD |
| 27 | Mechanical Integration: Dual SO-101 Arms | <span style="color: green">Done</span> | Assembled dual SO-101 arms; verified reach/clearance; added power budget/fusing direction for arms; harness routing and strain relief; updated CAD and pin/power map. | 2026-05-07 | 2026-02-08 |
| 28 | AMR Base Hardware Completion | Planned | Finish the base robot before manipulator expansion: 4x proximity sensor mounts/wiring with final STM pin selection, STM proximity driver path, IMU dev-board wiring to planned STM I2C, INA226 + external shunt battery monitor wiring to planned STM I2C, camera/depth mounting cleanup, cable strain relief, power/USB stability checks, and docs/pin-map updates. Battery monitor target is total pack voltage/current/power after the main fuse and before branch distribution. Keep this focused on making the AMR base reliable and inspectable. | 2026-05-14 | TBD |
| 29 | AMR Base Reliability + Sensor Fusion | Planned | Bring up reliable `/imu` if available, define EKF ownership, validate wheel odom/IMU/scan timing, retest AMCL/Nav2, investigate localization jumps, and run longer battery-powered routes. Add battery voltage/current/power telemetry to runtime monitoring and acceptance reports so low voltage, voltage sag, current spikes, and brownout risk are visible during tests. Use current sensing and odom evidence to characterize launch transients, slip, and drift. | 2026-05-21 | TBD |
| 30 | Orin NX AMR Runtime Migration | Planned | Move AMR driving toward Jetson Orin NX sooner: create a separate Orin Docker/runtime profile, likely Humble on JetPack 6, while keeping the current Foxy/Nano workflow as the validated fallback. Validate STM32 micro-ROS, LiDAR, RealSense, IMU, cameras, GPU, USB, thermals, network, and container device access before treating Orin as the primary AMR computer. | 2026-05-28 | TBD |
| 31 | Agentic AMR Operation Baseline On Orin | <span style="color: goldenrod">In Progress</span> | Turn the agentic baseline into an operating workflow for the base AMR on the Orin path: validate skills, harnesses, shared ROS clients, state-inspection MCP, guarded mission-control MCP, guarded robot-launch MCP, and voice/conversation/speaker MCPs; add software-only fixture tests for MCP/clients; harden confirmation-required command tools for `go_to_named_place`, `cancel_mission`, and supervised launch without bypassing mission/safety/Nav2. | 2026-06-04 | TBD |
| 32 | Supervised AMR Hardware Acceptance On Orin | Planned | Use the hardware acceptance harness on the real AMR with Orin as the target runtime once device bring-up is ready: idle, post-motion, localization readiness, mission success, safety recovery, fault decode, and repeated named-place routes. Compare against the known Nano/Foxy baseline and produce reports before enabling command MCPs for regular operation. | 2026-06-11 | TBD |
| 33 | Local LLM + Monitoring On Orin | Planned | Run a small local orchestration LLM and monitoring stack on Orin for operator dialogue, tool selection, status explanation, system health, and MCP calls. Keep deterministic parser paths for known commands. Enforce skills, permissions, confirmation, and AMR MCP safety gates before any motion-causing tool call. | 2026-06-18 | TBD |
| 34 | URDF/MoveIt For Base + Arms | Planned | Add SO-101 URDF/Xacro + collision meshes; integrate with base URDF/TF tree; calibrate arm base, tool, gripper, AMR camera, and wrist-camera frames; generate MoveIt2 configs and limits; verify planning scene in simulation/bench-safe mode. | 2026-06-25 | TBD |
| 35 | Arm Control Bring-Up With MoveIt | Planned | Bring up arm drivers through `ros2_control`, joint states, trajectory action or equivalent IO path, gripper control, homing, limits, soft stops, and basic guarded named poses. Keep execution bench-focused and approval-gated. | 2026-07-02 | TBD |
| 36 | Manipulation Perception + Calibration | <span style="color: goldenrod">In Progress</span> | Use AMR-mounted depth camera for broad scene/object discovery and wrist camera for close-range grasp refinement. Complete hand-eye calibration, base-to-arm extrinsics, camera-to-tool frames, AprilTag checks, RGB-D logging, and stale-frame rejection before grasp execution. | 2026-07-09 | TBD |
| 37 | Classical Grasp Pipeline With MoveIt | Planned | Build the first measurable manipulation pipeline: perception -> object/grasp proposal -> MoveIt reachability/collision planning -> guarded execution -> retreat/recovery -> wrist-camera verification. Use this as the baseline before VLA/VLM action policies. | 2026-07-16 | TBD |
| 38 | Base + Arm Integration Demo | Planned | Navigate to a named pickup pose, align to workspace, run classical grasp, place at drop zone, report status, and recover from failed grasp or navigation errors. Maintain consistent base/arm/camera transforms and safety watchdogs. | 2026-07-23 | TBD |
| 39 | External MCP Integrations For Home Autonomy | Planned | Integrate useful non-motion MCPs with the Orin LLM agent: Home Assistant for home state/devices, system monitoring for Orin health, calendar/tasks/reminders, and recipe/meal-planning or shopping-list MCPs. Keep robot motion behind AMR-owned MCP safety tools only. | 2026-07-30 | TBD |
| 40 | AMR Perception MCPs + Household Inspection | Planned | Add custom AMR perception MCP tools for camera inspection workflows such as workspace/fridge/counter scan, object list proposals, stale-frame checks, and operator confirmation. Connect outputs to recipe/task/home MCPs where useful, but do not let perception directly command actuators. | 2026-08-06 | TBD |
| 41 | VLM/VLA Trials For Manipulation | Planned | Profile VLM/VLA models on Orin and/or external GPU fallback. Use them for object discovery, grasp/action proposals, and scene inspection while MoveIt and guarded controllers remain responsible for planning/execution. Compare against the classical grasp baseline. | 2026-08-13 | TBD |
| 42 | Agentic Home Autonomy Demo | Planned | Demonstrate an agentic workflow that combines AMR operation, LLM orchestration, MCP integrations, and guarded manipulation: inspect a scene, suggest an action or meal/task, schedule/remind through calendar/task MCPs, optionally navigate or manipulate through AMR-owned safety-gated MCP tools, and log all decisions/confirmations. | 2026-08-20 | TBD |
| 43 | Productization Scope Freeze | Planned | Freeze one product-like showcase workflow: fixed-place indoor delivery/navigation with human load/unload. Define what is in scope, what is explicitly out of scope, known limitations, demo route, operator workflow, and acceptance criteria. Do not add broad autonomy features unless they improve this workflow. | 2026-08-27 | TBD |
| 44 | Repo Productization Baseline | Planned | Add top-level `README.md`, `Makefile`, `.env.example`, structured docs index, and cleanup plan for generated artifacts/placeholders. Convert `useful commands.md` into documented one-command workflows. Separate dev, ops, diagnostics, calibration, and hardware-acceptance scripts. Add the agentic robotics documentation baseline: agent roles, tool permissions, MCP wrapper plan, repo-local skills plan, and interaction examples. | 2026-09-03 | TBD |
| 45 | Automated Test Foundation | Planned | Add unit tests for `amr_voice` parser, mission config loading, mission request validation, and safety-supervisor health evaluation. Add `colcon test` workflow locally. Start with software-only tests that do not need hardware. Add the first agent harness scenarios for unsafe motion refusal, localization-not-ready denial, safety-fault denial, and manipulator plan-before-execute behavior. | 2026-09-10 | TBD |
| 46 | CI/CD Foundation | Planned | Add GitHub Actions or equivalent CI that runs formatting/lint checks, Python unit tests, ROS package build, and selected launch/config smoke checks on every PR/branch. CI should block regressions before hardware testing; CD remains limited to tagged Docker/image/release artifacts until deployment is safer. | 2026-09-17 | TBD |
| 47 | Hardware Acceptance & Reliability Report | Planned | Turn baseline probes into a repeatable hardware acceptance suite: idle, motion, post-motion, localization readiness, fault decode, safety recovery, and mission success criteria. Run 20+ repeated delivery/navigation missions and summarize success rate, faults, localization jumps, recovery time, and known limitations. | 2026-09-24 | TBD |
| 48 | Product-Grade Demo Package | Planned | Produce a polished productization case study: operator demo, deployment guide, safety case, acceptance-test report, architecture diagram, BOM/cost estimate, release tag, and demo video. Present the AMR as a product-grade indoor delivery platform showcase, not a general-purpose robot claim. | 2026-10-01 | TBD |

> Canonical view rule: If the table and task board ever conflict, the table wins for schedule; task board wins for day-to-day details.

---

## Status Summary
- Overall: SLAM + localization + navigation pipeline is working on real hardware (slam_toolbox map -> saved map -> AMCL -> Nav2 goals); one-shot Jetson+dev-PC bring-up scripts now include STM reset through ST-LINK/OpenOCD; the mission layer now runs as a persistent dev-PC-side runtime on top of Nav2 with typed status/state and integrated tmux panes; SO-101 integration and an initial ACT manipulation demo are now validated; low-level control/odometry calibration is still in active tuning.
- Progress: 23/48 weeks complete (~48%) (plan is now being executed iteratively vs. strictly week-by-week, with Weeks 43-48 reserved for productization maturity).
- Recent:
  - Reflashed `STM_Firmware_AMR_v2`, restored STM32 micro-ROS connectivity, and revalidated live `/amr_stm/*` topics on the Jetson bring-up path.
  - Re-checked wheel encoders on live `/amr_stm/wheel_state`, corrected left/right motor-channel wiring, and fixed wheel direction polarity so individual left/right wheel commands now actuate the intended side.
  - Added ROS current topics (`/amr_stm/current_left_ma`, `/amr_stm/current_right_ma`) plus raw ADC/zero topics for bench bring-up and calibration of the replacement current sensors.
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
  - Hardened the mission/Nav2 command path so mission CLI calls use unique ROS node names, wait only for the required service, and time out cleanly instead of hanging; `go_to kitchen` was revalidated through the mission server on real hardware.
  - Completed the real motor-driver power-cut safety recovery validation without restarting the safety supervisor; reset is rejected while STM fault bits are active, succeeds after fault clear, and STM re-enable remains manual.
  - Added a guarded operator recovery helper (`scripts/amr_safety_recover.py`) so the validated safety recovery sequence can be run repeatably without manually typing every ROS command.
  - Integrated SO-101 into the active project stack and completed the main AMR mechanical assembly.
  - Ran an ACT policy that successfully picked up an object and placed it into a bag.
- micro-ROS: STM32 bring-up on USART2 with full AMR topic set live (wheel_state + duty topics visible); UART telemetry disabled to avoid contention.
- Safety: Hardware e-stop GPIO integrated with debounce and fault latch.
- Current Focus (next sprint):
  - Finish the AMR base as a reliable hardware/software platform before expanding manipulation: 4x proximity sensor hardware/driver path, IMU dev-board path, INA226 + external shunt battery monitoring, camera/depth mounting cleanup, cable/power/USB checks, and updated pin-map/docs.
  - Move AMR driving to Jetson Orin NX sooner rather than treating Orin only as a later AI add-on. Build this as a separate runtime migration: Orin/Humble container profile first, device access validation second, then AMR driving acceptance against the existing Nano/Foxy baseline.
  - Bring up a trustworthy `/imu` source before EKF work. Validate rate, timestamps, frame ID, mounting, covariance, and orientation/gyro stability before feeding `robot_localization`.
  - Keep EKF/localization improvement as the next navigation-quality track; the 2026-05-03 mission validation still showed multiple large AMCL pose steps during motion even though STM comms, wheel state, odom, scan, and safety supervision stayed healthy.
  - Turn the agentic baseline into a real AMR operating workflow: validate skills/harnesses/shared clients/read-only MCP, add fixture-backed MCP tests, then design confirmation-required command MCP tools that use mission/safety/Nav2.
  - Continue voice-command work as the operator-input path for agentic operation, with mission services as the backend for `go home`, `go kitchen`, `go hall`, `stop`, and `status`.
- Productization Direction:
  - Use this AMR as a productization case study: show how a prototype robotics workbench becomes a repeatable, testable, operator-friendly indoor delivery/navigation platform.
  - Product-grade does not mean certified or mass-manufactured yet. For this project phase it means: one-command build/run paths, clean documentation, automated tests, CI, explicit safety behavior, repeatable hardware acceptance, operator-facing workflow, release tags, and known limitations.
  - Keep the productized showcase narrow: fixed-place indoor delivery/navigation with human load/unload. Manipulation, VLA/VLM autonomy, custom PCB, and general-purpose robot behavior stay as longer-term R&D unless a validated workflow demands them.
- Agentic Tooling Direction:
  - Treat MCP servers, skills, subagents, and harnesses as an operator/developer interface above the existing ROS and firmware stack, not as a replacement control system.
  - Current baseline now includes agent contracts, permission rules, codebase ownership, repo-local skills, source-only harness checks, harness CI, shared ROS clients, a read-only AMR state MCP server, and an agentic behavior diagram.
  - Use nine working agents for current routing: test runner, code review, STM firmware, ROS core/hardware interface, navigation/mission/safety, manipulator/MoveIt, perception/calibration, voice/operator interface, and runtime environment.
  - Continue with read-only and diagnostic capabilities first: robot health, safety state, mission state, named places, localization status, logs, and fault summaries.
  - Add motion-causing tools only after software-only fixtures, command preconditions, safety prechecks, explicit confirmation, audit logging, and supervised hardware acceptance procedures are in place.
  - On Orin, run a small local LLM for orchestration/tool selection and keep VLM/VLA workloads on demand for perception/manipulation proposals. External MCPs such as Home Assistant, system monitoring, calendar/tasks, reminders, and recipe/meal planning may provide context or non-motion actions, but robot motion must remain behind AMR-owned MCP safety tools.
  - Keep agent tools intent-level (`go_to_named_place`, `cancel_mission`, `plan_arm_named_pose`) and block raw motor/PWM/unguarded joint-control pathways.
- Control Architecture Direction:
  - Yes, moving toward cascaded control makes sense: inner current/torque limiting (or current loop if feasible) + outer speed loop is the standard industrial structure and will reduce slip/launch transients once current sensing is reliable.
- Next Focus:
  - AMR base completion and supervised acceptance.
  - Orin NX AMR runtime migration while keeping Nano/Foxy as the known-good fallback until Orin passes hardware acceptance.
  - Agentic AMR operation through voice/text/LLM -> skill -> MCP -> shared ROS client -> mission/safety/Nav2.
  - MoveIt-first manipulator bring-up after the AMR base is stable.
  - Orin LLM/MCP integrations after the Orin AMR runtime and safety boundaries are proven.
- Timeline: Flexible (now a 48-week plan with extensions). Prioritize AMR base reliability plus Orin runtime migration first, agentic AMR operation second, MoveIt manipulation third, and broader Orin-based autonomous integrations after the core robot is reliable; custom PCB is low priority/optional unless required for a pilot or repeatable demo.

## Calendar Baseline (Week Alignment)
- Week 1 start: 2025-10-17.
- Paused weeks (approx): 2025-11-03 to 2025-11-09 and 2025-11-17 to 2025-11-23.
- Plan weeks are working weeks; paused weeks extend the calendar timeline by ~2 weeks. Update these dates if the break windows were different.
  - Firmware Branching: v1 (bench, L298N + small encoder) is now frozen; all new work proceeds in v2 (Cytron MDD20A + post-gearbox encoder).

Legend: <span style="color: green">Done</span>, <span style="color: goldenrod">In Progress</span>, Partial, Planned, Blocked

---

## Productization Track

The AMR will be used as a productization showcase, not only an R&D platform. The objective is to demonstrate the engineering discipline needed to turn a working robot into something another engineer/operator can build, test, run, debug, and evaluate.

### Productization Goal

Productize one narrow scenario first:

```text
Indoor fixed-place delivery/navigation robot:
- map a known indoor environment
- define named places
- send the robot to a selected place
- human loads/unloads payload
- robot reports status and returns home
- logs mission result
- exposes clear safety/recovery behavior
```

This avoids claiming a general-purpose robot too early while still producing a strong product-grade demo.

### What Product-Grade Means For This Repo

- Clear top-level `README.md`: current capability, hardware assumptions, build/run/test instructions, known limitations.
- One-command workflows: `make build`, `make test`, `make nav`, `make safety-baseline`, `make hardware-check` or equivalent scripts.
- Automated software tests for deterministic logic: voice parser, mission config loading, mission validation, safety health-state logic.
- CI checks on every branch/PR: build, tests, lint/static checks, package metadata, and smoke checks that do not require hardware.
- Hardware acceptance suite: repeatable idle/motion/post-motion probes, mission success criteria, safety recovery checks, and saved reports.
- Agent-facing interface discipline: documented agent roles, project skills, MCP tool permission levels, structured outputs, and harness scenarios for unsafe requests.
- Deployment docs: Jetson/dev-PC setup, map creation, place calibration, startup, shutdown, log collection, recovery.
- Safety case: hazards, mitigations, current limitations, operator responsibilities, and what is not certified.
- Release discipline: tag stable demo states, record test results, keep generated logs/build artifacts out of source.

### Agentic Tooling Track

Detailed task breakdown: `docs/agentic/agentic_robotics_roadmap.md`.

This track introduces MCP servers, subagent roles, repo-local skills, shared ROS clients, and agent harnesses as structured tooling around the AMR. The first milestone is now a working documentation and read-only diagnostics baseline. Motion-causing tools must remain behind explicit confirmation, safety-state checks, localization checks, audit logging, fixture-backed tests, and supervised hardware acceptance.

Implemented baseline:

- Defined nine agent roles, including the runtime environment agent for Docker, Jetson Nano, and Orin NX work.
- Added repo-local skills for test running, code review, STM firmware, ROS core/hardware, navigation/mission/safety, runtime environment, voice, perception, manipulator bring-up, navigation debug, mission runtime, safety recovery, and hardware acceptance.
- Added `agent_harness` source-only checks and GitHub CI for harness validation.
- Added shared `amr_clients` ROS client helpers so CLI, voice, and MCP paths do not duplicate mission or safety logic.
- Added the read-only `mcp_servers/amr_state_inspection` server for robot health, safety state, mission state, named places, localization, STM diagnostics, and navigation state.
- Added `docs/agentic/agentic_behavior_diagram.md` to show how users, agents, skills, harnesses, MCPs, shared ROS clients, and the robot runtime fit together.

Next work:

- Add software-only functional validation for skills, shared ROS clients, and MCP behavior without requiring a physical AMR.
- Add fake-ROS or fixture-backed MCP integration tests so read-only MCP behavior can be checked in CI.
- Add confirmation-required command MCP tools only after guardrails are documented and tested: go to named place, cancel mission, recovery checks, and later approved arm plans.
- Add supervised hardware acceptance for the AMR base before treating command MCPs as a regular operating path.
- Bring up Orin as the near-term AMR runtime and later on-robot LLM/MCP runtime: separate Docker profile, system monitoring, local small LLM orchestrator, explicit Foxy/Nano vs Humble/Orin separation, and hardware acceptance before Orin becomes the primary driving computer.
- Integrate external MCPs only after the AMR-owned safety boundary is clear. First candidates: Home Assistant for home state, system monitoring for Orin health, calendar/task/reminder MCPs, and recipe/meal-planning MCPs. These should provide context or non-motion actions; AMR motion stays behind AMR-owned MCP tools.
- Keep subagent workflow PR-based: domain agent performs bounded change, Test Runner Agent validates, Code Review Agent reviews, then merge through PR.
- Expand supervised hardware acceptance reports only when the physical AMR is available and the operator explicitly requests a hardware run.

### Productization Guardrails

- Do not add new autonomy features unless they strengthen the fixed-place delivery/navigation showcase.
- Keep dual-arm manipulation as a future capability until navigation, safety, logging, and operator workflow are repeatable.
- Treat every recurring manual command as a candidate for a script, Make target, or documented operator procedure.
- Treat every hardware validation as a future acceptance test with pass/fail criteria.
- Document limitations honestly. Known limitations increase credibility when they are measured and bounded.

## CI/CD Notes

CI/CD means Continuous Integration and Continuous Delivery/Deployment.

### Continuous Integration

CI is the habit of checking every code change automatically before it reaches the main working branch. In this AMR project, CI should answer:

```text
Did this change break the ROS workspace build?
Did parser/mission/safety logic still pass tests?
Are launch/config files still valid enough for smoke checks?
Did package metadata or dependencies regress?
```

CI is needed because robotics systems have many layers: firmware, ROS packages, launch files, YAML configs, Docker images, scripts, maps, and hardware assumptions. A small change to a topic name, QoS profile, service timeout, or YAML key can silently break a real robot. CI catches the software-side mistakes before risking hardware time.

Initial CI for this repo should be software-only:

1. Checkout the repo.
2. Install/build the ROS dependencies or use the existing Docker image.
3. Run `colcon build`.
4. Run `colcon test`.
5. Run Python unit tests for deterministic packages.
6. Run lightweight lint/static checks.
7. Optionally run launch/config smoke checks that do not require Jetson, STM32, LiDAR, or motors.

Hardware-in-the-loop tests come later because they require the physical robot and operator supervision.

### Continuous Delivery

CD means producing a repeatable release artifact after CI passes. For this AMR, early CD should mean:

- tagged Docker images for dev-PC/Jetson runtimes
- versioned release notes
- packaged config profiles
- saved acceptance-test reports
- release tags such as `v0.1.0-navigation-demo` or `v0.2.0-productized-delivery-demo`

This is safer than automatic deployment. The robot should not auto-update and run on hardware just because code was pushed.

### Continuous Deployment

Continuous Deployment is when passing changes are automatically deployed to production. For this AMR, that should be avoided until the platform has strong rollback, remote supervision, safety gates, and field maturity. Physical robots can damage hardware or create safety risks, so deployment should stay manual and deliberate for now.

### CI/CD Roadmap For AMR

- Phase 1: Add local tests and `make test`.
- Phase 2: Add CI for unit tests and `colcon build`.
- Phase 3: Add launch/config smoke checks.
- Phase 4: Add Docker image build checks.
- Phase 5: Add tagged release artifacts and release notes.
- Phase 6: Add hardware acceptance reports, still manually triggered.
- Phase 7: Consider supervised deployment/update flow only after rollback and safety procedures are mature.

---

## PCB Migration Milestones (dev boards -> integrated AMR control PCB)

PCB migration remains a later optional productization track. It should not block the immediate AMR base reliability, agentic operation, MoveIt manipulation, or Orin MCP/LLM integration work unless wiring reliability or repeatability becomes the limiting factor.

- Conceptual architecture (target end-state): Define final board contents (STM32/bare MCU or SOM, dual motor stage, buck rails 12->5->3.3 V, encoder conditioning, current sense, battery protection, all IO connectors, CAN/UART/RS485/I2C). This is the "Cytron + Nucleo + buck + encoder + UART + IO" rolled into one.
- Phase A - Measure on dev boards: With Nucleo + Cytron + XL4016, capture real currents (continuous/peak), encoder voltage tolerance/noise, UART/CAN bandwidth, EMI/ground noise patterns, ADC resolution needs, harness lengths and connector types. This produces the electrical requirements doc.
- Phase B - Consolidation carrier PCB: Keep Nucleo + Cytron + external buck, but design a carrier/backplane that handles connectors, power distribution, current sensing, ferrite/filter caps, and ground planes to organize wiring and validate signal/power integrity.
- Phase C - Integrate motor driver: Drop Cytron; design your own dual H-bridge with gate driver (e.g., DRV87xx) + MOSFETs + shunts/Hall sensors. Validate thermals, switching, protection (TVS/fuse/reverse), and EMC.
- Phase D - Integrate MCU: Replace Nucleo with bare STM32F401 (LQFP-64): crystal, boot config, SWD header, decoupling, ESD/TVS, brownout protection. Port firmware; bring-up clocks, debug, boot, and motor control.
- Phase E - Production/EMC-ready: Move to 4-layer, split/stitched grounds, Kelvin sensing, star grounds, TVS + common-mode chokes, reverse/fuse protection, panelization notes, silks/labels, test points. Run pre-scan EMC/ESD and pilot build (5-10 boards) with harnesses and acceptance tests.
---


## Architecture Docs
- ROS stack diagrams: `docs/architecture/ros_stack_diagrams.md`
- STM32 firmware architecture: `docs/architecture/STM_architecture.md`
- Jetson Nano runtime architecture: `docs/architecture/jetson_architecture.md`
- Agentic behavior diagram: `docs/agentic/agentic_behavior_diagram.md`
- Agentic robotics roadmap: `docs/agentic/agentic_robotics_roadmap.md`
- Agent tool permissions: `docs/agentic/agent_tool_permissions.md`
- Agent interaction examples: `docs/agentic/agent_interaction_examples.md`
- Initial agent role contracts: `docs/agentic/roles/`


---

## Project Log
- 2026-05-10: Reworked the weekly roadmap to include the new agentic-AI robot development sequence discussed over the last two days. The near-term order is now AMR base completion, proximity/IMU/sensor-fusion reliability, agentic AMR operation through skills/MCP/shared ROS clients, supervised hardware acceptance, Orin runtime and local LLM orchestration, MoveIt-first manipulator bring-up, and then external MCP integrations such as Home Assistant, system monitoring, calendar/tasks/reminders, and recipe/meal planning.
- 2026-05-10: Adjusted the roadmap to move Jetson Orin NX into the near-term AMR driving path. Orin runtime migration now happens before regular command MCP operation: build a separate Orin/Humble container profile, validate hardware devices and ROS contracts against the existing Nano/Foxy baseline, then run supervised AMR hardware acceptance on Orin before making it the primary robot computer.
- 2026-05-10: Added battery monitoring to the near-term AMR base hardware tasks. Planned hardware path is INA226 over STM I2C `PB8/PB9` with an external 50 A minimum, preferably 75/100 A, shunt placed after the main fuse and before branch power distribution. IMU and INA226 share the planned STM I2C bus; firmware/CubeMX changes are still required. Proximity pins are proposed-final but still require CubeMX/header verification. The voltage display remains display-only, and the battery 5-pin JST-XH connector remains unassigned until pinout is verified.
- 2026-05-10: Updated the agentic implementation sequence to reflect the current baseline: agent contracts, ownership, skills, harness, harness CI, shared ROS clients, read-only AMR state MCP, and agentic behavior diagram are now implemented baseline items. The next agentic focus is software-only functional validation, fixture-backed MCP tests, and only then confirmation-required command MCP tools.
- 2026-05-09: Marked `docs/project/AMR_firmware_tasks.tmp` obsolete and removed its stale legacy references to missing planning docs. The canonical firmware/project status now lives in `docs/project/AMR_project.md`, `docs/architecture/STM_architecture.md`, `docs/hardware/pin_map.yaml`, and the current firmware source.
- 2026-05-09: Created the first two repo-local agent skills: `.codex/skills/amr-code-review/SKILL.md` and `.codex/skills/amr-test-runner/SKILL.md`. These skills reference the detailed role contracts and provide practical workflows, safe command boundaries, subsystem selection guidance, and standardized output formats for review and validation work.
- 2026-05-09: Refined the agent role structure from a compressed starter set into eight target agents: test runner, code review, STM firmware, ROS core/hardware interface, navigation/mission/safety, manipulator/MoveIt, perception/calibration, and voice/operator interface. Added contracts for the new split while keeping manipulator, perception, and voice contracts intentionally lighter until those subsystems mature.
- 2026-05-09: Started Phase 1 of the agentic robotics plan by adding the first agent permission model, interaction examples, and role contracts for the test runner, code review, ROS/mission/safety, and STM firmware agents. These documents define allowed commands, blocked commands, required checks, done criteria, failure modes, and escalation rules before skills or MCP tools are implemented.
- 2026-05-09: Added an agentic robotics roadmap for using MCP servers, subagent roles, repo-local skills, and harness scenarios as a structured operator/developer interface above the existing ROS 2, Nav2, MoveIt2, micro-ROS, and STM32 control stack. Updated the productization roadmap to include agent tool permissions, read-only-first MCP diagnostics, confirmation-required motion tools, engineering agents for test/review/code generation across STM firmware, ROS 2, voice, and perception, and safety-focused harness scenarios.
- 2026-05-04: Added a productization track to the canonical AMR roadmap. The AMR will be treated as a productization case study for a narrow fixed-place indoor delivery/navigation workflow, with repo maturity milestones for README/Makefile cleanup, automated tests, CI/CD foundation, hardware acceptance, reliability reporting, safety case, deployment docs, and a product-grade demo package. Added CI/CD notes explaining why CI is needed for robotics, how it should start as software-only checks, and why automatic deployment to physical robots should remain manual/supervised until rollback and safety procedures are mature.
- 2026-05-03: Hardened the Voice Command MVP for no-speaker operation. Voice motion commands now refuse to proceed until `map -> odom` localization is available, `/amr_voice/feedback` publishes operator-facing text for future TTS, and `scripts/open_amr_voice_asr.sh` launches laptop-mic ASR with the correct workspace setup.
- 2026-05-03: Started Voice Command MVP Step 3. Added `voice_asr_node`, a laptop-microphone Vosk ASR node that publishes `/amr_voice/transcript` and `/amr_voice/partial_transcript`, supports device listing and dry-run mode, and feeds final transcripts through the existing wake-gated mission command handler.
- 2026-05-03: Started Voice Command MVP Step 4. Motion commands now require an explicit `yes` confirmation before calling mission services, while `stop` / `cancel` still bypass confirmation so safety stops are never blocked. Added rejection handling with `no` / `cancel that` and documented the ASR confirmation flow.
- 2026-05-03: Completed Voice Command MVP Step 2 in typed-text form. The voice command node now supports wake-gated operation: `lovely` opens a short listening window, commands with the wake word execute immediately, and stop/cancel remains accepted without the wake word. The one-shot navigation tmux workflow now starts the voice pane with wake gating enabled.
- 2026-05-03: Started Voice Command MVP Step 1. Added the `amr_voice` ROS package with a deterministic text-command parser and `voice_text_cli` / `voice_command_node` entry points. Supported typed intents now include go-to-place (`home`, `door`, `kitchen`, `hall`), stop/cancel, status, and list-places; the package builds in `amr_devpc` and dry-run parsing was validated.
- 2026-05-03: Marked Week 25 Safety Supervision & Health Monitoring complete after the merged safety-supervision branch and validated motor-driver power-cut recovery flow. Remaining safety improvements should be tracked as follow-up tasks rather than keeping the baseline safety layer open.
- 2026-05-03: Reprioritized the weekly plan after safety-supervision merge. Cascaded PID/current-loop work is marked complete for this phase with production motion staying on the validated single-loop speed controller; proximity sensors are returned to planned status; voice-command control is now the active branch and current sprint, split into text parsing, "lovely" wake-word flow, laptop ASR, confirmations, Jetson speaker feedback, full loop, and robustness guards.
- 2026-05-03: Completed Safety Step 10: repeated the real motor-driver power-cut fault while moving, confirmed STM latched `STALL_LEFT|STALL_RIGHT` (`fault_mask=24`) with comms healthy, verified `/amr/safety_supervisor/reset_intervention` rejects while unsafe, restored motor-driver power, cleared STM faults, reset the supervisor without restarting it, manually re-enabled STM, and finished with a passing 30 second baseline probe.
- 2026-05-03: Added guarded safety recovery tooling. `scripts/amr_safety_recover.py` cancels mission/Nav2 motion, publishes zero velocity, disables STM, decodes live STM/comm/supervisor state, prompts before clearing nonzero STM faults, calls `/amr/safety_supervisor/reset_intervention`, and keeps STM re-enable as an explicit operator step unless requested with `--reenable`. Dry-run validation passed on healthy hardware.
- 2026-05-03: Hardened the mission/Nav2 command path after CLI calls intermittently hung or failed service discovery. `mission_cli` now uses a unique ROS node name per invocation, waits only for the service required by the requested command, and bounds service-response waits. `mission_server` now clears stale action goal handles on idle cancel requests. Rebuilt/restarted the mission server and validated `status`, `cancel`, and `go_to kitchen` on real hardware. Follow-up: improve localization/AMCL behavior because mission validation still showed large AMCL pose steps during motion while low-level comms and safety remained healthy.
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
- 2025-12-22: Integrated mechanical CAD tasks into Week 17; renamed STM architecture doc to `docs/architecture/STM_architecture.md`; removed merged task/architecture files.
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

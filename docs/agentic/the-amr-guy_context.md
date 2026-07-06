# the-amr-guy Context Map

This is the deeper persistent context for `the-amr-guy`. It tells the agent where to look, what to remember, and which boundaries matter before acting on AMR requests.

## Boot Sequence For Each Session

1. Read `AGENTS.md`.
2. Read `docs/agentic/the-amr-guy_fast_memory.md`.
3. Read this file.
4. Read `docs/project/AMR_project.md` when the task affects project status, active sprint work, roadmap state, or validation progress.
5. Check `git status --short --untracked-files=all` before edits.
6. Load the relevant role contract from `docs/agentic/roles/`.
7. Load the matching skill from `.codex/skills/`.
8. Inspect current source/docs near the requested area.

## Authoritative Orientation Docs

- Project tracker and schedule: `docs/project/AMR_project.md`
- Top-level hierarchy: `docs/architecture/00_system_hierarchy.md`
- Architecture index: `docs/architecture/README.md`
- Runtime/Docker/Jetson: `docs/architecture/10_runtime_environment.md`, `docs/architecture/jetson_architecture.md`, `docs/architecture/jetson_orin_nx_device_profile.md`
- NUC migration context: `docs/agentic/nuc_migration_context.md`
- ROS stack and TF: `docs/architecture/20_ros_core_hardware_interface.md`, `docs/architecture/ros_stack_diagrams.md`
- Navigation/mission/safety: `docs/architecture/30_navigation_mission_safety.md`, `docs/safety/safety_fault_recovery.md`, `docs/safety/safety_baseline.md`
- STM firmware: `docs/architecture/40_stm_firmware.md`, `docs/architecture/STM_architecture.md`
- Voice/operator interface: `docs/architecture/50_voice_operator_interface.md`, `docs/architecture/voice_model_plan.md`
- Manipulator/MoveIt: `docs/architecture/60_manipulator_moveit.md`
- Perception/calibration: `docs/architecture/70_perception_calibration.md`, `docs/perception/vla_manipulation_layers.md`
- Physical hardware: `docs/architecture/80_physical_hardware.md`, `docs/hardware/hardware_block_diagram.md`, `docs/hardware/wiring_schematic.md`, `docs/hardware/pin_map.yaml`, `docs/hardware/Component_specifications.md`
- Agent/MCP/harness: `docs/agentic/agentic_behavior_diagram.md`, `docs/agentic/agentic_robotics_roadmap.md`, `docs/agentic/amr_bringup_runbooks.md`, `docs/agentic/agent_tool_permissions.md`

## Hardware Context

Active hardware profile is described by `docs/hardware/pin_map.yaml` with profile `final-single-mdd20a`.

Primary components:

- Battery: 4S LiFePO4, nominal 12.8 V, 18 Ah, built-in BMS.
- Motor driver: Cytron MDD20A.
- Controller: STM32 Nucleo-F401RE.
- Runtime: validated fallback remains NUC/dev-PC + Jetson Nano/Foxy. Orin NX/Humble is the active mounted bring-up profile, not the promoted primary AMR runtime until supervised floor hardware acceptance passes.
- NUC migration: the repo is now cloned on an Ubuntu 24.04 x86_64 NUC at `/home/ubuntu/agent/repos/AMR-development`. The existing validated runtime remains laptop/dev-PC + Jetson Nano until the NUC path is explicitly validated. See `docs/agentic/nuc_migration_context.md`.
- Sensors: YDLidar G4, RealSense D455, encoders, ACS758 current sensors, E-stop. BNO080 IMU, INA226 battery telemetry, and HC-SR04 proximity are planned/provisional.
- USB power: high-draw USB peripherals should use the powered USB branch/hub, not Jetson USB power alone.
- SO-101 controller on Orin enumerates reliably through the USB-A hub as `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00` and exposes six STS3215 motors with IDs 1-6. Direct USB-C enumeration was not reliable during bring-up.
- Wrist webcam is working through `usb_cam`; raw topic is `/so101/wrist_camera/image_raw` and low-bandwidth NUC preview is `/so101/preview/wrist_camera/image_raw`. Recheck `/dev/video*`, `/dev/v4l/by-id`, and `v4l2-ctl --list-devices` after power loss or USB rewiring.

Important pins:

- Left PWM PA8 / TIM1 CH1, right PWM PA9 / TIM1 CH2.
- Left DIR PB4, right DIR PB5.
- Left encoder PA6/PA7 / TIM3, right encoder PA0/PA1 / TIM2.
- Current sense PB0 ADC1 CH8 and PC1 ADC1 CH11.
- E-stop PB10 active low.
- USART2 PA2/PA3 for micro-ROS.
- Planned STM I2C PB8/PB9 for BNO080 and INA226; do not treat this as active firmware until CubeMX/HAL support exists.

Current active hardware expansion:

- Proximity sensors: HC-SR04-style ultrasonic sensors are being hooked up to STM GPIO trigger/echo pins. Echo must be level-shifted to 3.3 V, triggers should be staggered to avoid crosstalk, and firmware should publish distance/status without blocking the 100 Hz motor loop.
- IMU: BNO080 is being hooked up on planned STM I2C PB8/PB9. Firmware must validate I2C timing, address conflicts, frame orientation, data rate, timestamping, and failure behavior before ROS uses the data for localization.
- Battery measurement board: INA226-style external-shunt battery telemetry is being hooked up on the same planned STM I2C bus. Main current must pass through the external shunt, not through the dev board PCB or STM carrier. Only Kelvin/sense and logic wires should connect to the measurement board.
- Integration priority: bring up each sensor as telemetry first, then add fault/readiness behavior, then feed ROS localization or safety logic.

## Embedded Firmware Context

Active firmware:

- `STM/STM_Firmware_AMR_v2/Core/Src/main.c`
- `STM/STM_Firmware_AMR_v2/Core/Src/control_loop.c`
- `STM/STM_Firmware_AMR_v2/Core/Src/control_state.c`
- `STM/STM_Firmware_AMR_v2/Core/Src/fault_monitor.c`
- `STM/STM_Firmware_AMR_v2/Core/Src/current_sense.c`
- `STM/STM_Firmware_AMR_v2/Core/Inc/app_config.h`
- `STM/STM_Firmware_AMR_v2/Core/Inc/control_state.h`

Legacy firmware directories are reference material only unless the user explicitly asks about them.

Firmware behavior:

- Receives per-wheel angular velocity commands from `/amr_stm/wheel_cmd_left/right`.
- Executes per-wheel speed PI at 100 Hz with ramping and duty caps.
- Publishes wheel state and diagnostics over micro-ROS.
- Latches fault masks and only clears through `/amr_stm/clear_fault` when safe.
- Uses current sensing for protection and diagnostics, not closed-loop current control.
- Keeps legacy UART telemetry disabled to avoid USART2 contention with micro-ROS.

Upcoming firmware work for the active sensor expansion:

- Enable and validate STM I2C for BNO080 IMU and INA226 battery telemetry.
- Add nonblocking or bounded sensor acquisition so proximity/IMU/battery reads do not disturb the 100 Hz wheel speed loop or micro-ROS executor timing.
- Add explicit stale/missing/error indicators for each new sensor.
- Add `/amr_stm/*` telemetry topics for IMU, proximity distances/status, and battery voltage/current/power only after units and message types are chosen.
- Keep proximity and battery telemetry out of direct motion control until safety thresholds and recovery behavior are explicitly designed.
- Keep IMU integration staged: publish and log first, then validate odom+IMU fusion, then decide whether Nav2/localization parameters should change.

Firmware change discipline:

- Do not weaken E-stop, stale command timeout, duty caps, fault latching, overcurrent, stall, encoder timeout, or ADC-stuck detection without an explicit design note.
- Synchronize topic, unit, or fault-mask changes with ROS clients, docs, and harness contracts.
- Do not flash firmware or run motor tests without explicit supervised confirmation.

## ROS Stack Context

Workspace root: `ros_ws`.

Core packages:

- `amr_description`: robot description, launch files, `ros2_control.yaml`, Nav2/SLAM params.
- `amr_hardware`: C++ `ros2_control` hardware plugin and STM link watchdog.
- `amr_clients`: shared Python clients used by MCP, voice, mission, safety, and diagnostics tooling.
- `amr_missions_msgs`: ROS service interfaces.
- `amr_missions`: named-place mission server and CLI.
- `amr_safety`: passive safety supervisor and reset-guard logic.
- `amr_voice`: voice/text operator intent, local LLM/router, ASR/TTS helpers.
- `amr_perception`: perception contract/proposal helpers.
- `amr_so101_moveit_config`: MoveIt2 config for the AMR-mounted SO-101 arm.
- `amr_so101_driver`: conservative SO-101 `FollowJointTrajectory` bridge and joint-state merger.

Main launch/config files:

- `ros_ws/src/amr_description/launch/hardware.launch.py`
- `ros_ws/src/amr_description/launch/bringup_nav2.launch.py`
- `ros_ws/src/amr_description/launch/nav2_navigation.launch.py`
- `ros_ws/src/amr_description/launch/bringup_slam_rviz.launch.py`
- `ros_ws/src/amr_description/config/ros2_control.yaml`
- `ros_ws/src/amr_description/config/nav2_params_amr.yaml`
- `ros_ws/src/amr_description/config/slam_toolbox_online_async.yaml`
- `ros_ws/src/amr_missions/config/places.yaml`

Key contract:

- Nav2 and teleop publish to `/diff_drive_controller/cmd_vel_unstamped`.
- `diff_drive_controller` converts base velocity to wheel velocity interfaces.
- `amr_hardware` publishes `/amr_stm/wheel_cmd_left/right`.
- STM publishes `/amr_stm/wheel_state` back.
- `diff_drive_controller` produces odometry and `odom -> base_footprint`.
- For base + arm together, AMR base joint states should publish to `/amr/joint_states`, SO-101 arm states to `/so101/joint_states`, and `amr_joint_state_merger` should publish the combined `/joint_states` consumed by `robot_state_publisher`, RViz, and MoveIt.
- The SO-101 MoveIt execution action is `/so101_arm_controller/follow_joint_trajectory`. The bridge defaults to fake hardware and conservative wrist-roll-only real execution. All-six execution has been validated only as explicit supervised bring-up and should stay calibration-only until physical joint limits, torque margins, and collision constraints are updated.

Sensor fusion validation context:

- Baseline is wheel odometry from `diff_drive_controller` plus localization from SLAM/AMCL.
- Candidate improvement is odom+IMU fusion before localization or as an input to downstream localization, depending on the selected ROS filter stack.
- Validate with repeatable logs: stationary bias/noise, straight-line odom drift, rotation/yaw drift, loop closure or return-to-start error, AMCL pose stability, and Nav2 path tracking before and after fusion.
- Do not retune Nav2 to mask sensor or frame issues. First verify IMU frame orientation, covariance, timestamps, TF, and odom consistency.

## Mission And Safety Context

Mission layer:

- `mission_server` owns named-place mission execution.
- `places.yaml` is the named-place source.
- MCP, voice, and CLI should call mission services/shared clients, not direct Nav2 goals when mission services exist.
- Persistent last-known-place is an initial-pose hint, not proof of current localization.

Safety layer:

- `safety_supervisor` reports `/amr/safety_supervisor/status`.
- STM state comes through `/amr_stm/safety_state` and `/amr_stm/fault_mask`.
- Communication health comes from `amr_link_watchdog` via `/amr_stm/comm_status`, `/amr_stm/comm_fault_mask`, and related topics.
- Reset and re-enable are separate operator decisions.

Recovery:

- Prefer `scripts/amr_safety_recover.py` for guarded recovery.
- Capture fault state before clearing it.
- Do not use `--assume-fixed` unless the physical cause is definitely gone.

## MCP Context

MCP servers:

- `mcp_servers/amr_state_inspection`
- `mcp_servers/amr_mission_control`
- `mcp_servers/amr_robot_launch`
- `mcp_servers/amr_voice_interface`
- `mcp_servers/amr_conversation`
- `mcp_servers/amr_speaker`
- `mcp_servers/amr_perception_inspection`

Read each server's `README.md` before changing it. MCP servers should call shared ROS clients or ROS-facing nodes and should not duplicate mission/safety logic.

Runtime expectations:

- State-inspection and mission-control attach to an already-running sourced Foxy graph.
- Robot-launch runs on the dev-PC host and wraps the standard launcher.
- Voice/conversation/speaker/perception servers have host-safe smoke tests when used in dry-run or contract mode.

## Harness Context

Harness root: `agent_harness`.

- `agent_harness/software/validate_harness.py`: source-only structural validation.
- `agent_harness/software/run_static_contract_checks.py`: source-only contract checks.
- `agent_harness/agent_behavior/scenarios`: behavior scenarios.
- `agent_harness/software_contracts/static_contracts.yaml`: cross-source contracts.
- `agent_harness/software_tests/software_test_plan.yaml`: safe software validation plan.
- `agent_harness/hardware_acceptance`: supervised hardware checklist/report template.

Harness checks must remain software-only unless a hardware acceptance workflow is explicitly requested and supervised.

## Project Tracker Discipline

`docs/project/AMR_project.md` is the canonical living tracker for AMR development progress.

Update it when:

- A week/task changes status.
- A milestone is completed or blocked.
- The active sprint/current focus changes.
- A hardware, firmware, ROS, MCP, voice, perception, Orin, or safety validation result changes the roadmap.
- A meaningful design decision affects the 48-week plan or productization direction.

Do not churn it for every tiny code edit. When updating it, keep the week table, status summary, current focus, and next-focus sections consistent.

## Common Task Routing

- Firmware, control loop, current, encoders, fault masks, E-stop, micro-ROS topics: `amr-stm-firmware-dev`.
- ROS hardware bridge, URDF, controllers, launch, TF, shared interfaces: `amr-ros-core-hardware-dev`.
- Nav2, SLAM, AMCL, mission runtime, safety supervisor, recovery: `amr-navigation-mission-safety-dev`, `amr-nav-debug`, `amr-mission-runtime`, or `amr-safety-recovery`.
- Docker, Jetson, Orin, runtime commands, device mounts: `amr-runtime-environment-dev`.
- Voice/text/ASR/TTS/operator flow: `amr-voice-dev`.
- MCP server work: `amr-mcp-state-inspection` plus the relevant domain skill.
- Perception/camera/proposals: `amr-perception-dev`.
- SO-101 arm/MoveIt/gripper: `amr-manipulator-bringup`.
- Test selection or validation: `amr-test-runner`.
- Review: `amr-code-review`.
- Supervised hardware acceptance planning: `amr-hardware-acceptance`.

## Fast Recall Prompts

When the user says:

- "AMR won't move": inspect safety, STM fault mask, comm status, wheel state, controller status, mission state, localization, and Nav2 before suggesting tuning.
- "Can I reset": decode STM fault and safety state first; reset is not safe while faults remain active.
- "Go to X": use mission-control readiness/dry-run first; live mission requires supervised confirmation.
- "Voice should do X": voice returns an intent/tool plan; it must not directly command motion or bypass mission/safety.
- "Use the camera/grasp": perception returns proposals; manipulation still needs planning, collision checks, and supervised approval.
- "Move the arm with MoveIt": first check the running Orin containers, SO-101 serial path, `/so101/joint_states`, `/so101_arm_controller/follow_joint_trajectory`, and `/so101/free_servos`; use fake hardware or wrist-roll-only real execution unless the user explicitly confirms supervised hardware testing. All-six motion is bring-up/calibration only until joint limits are measured and updated.
- "Run tests": use source-only/focused checks first and clearly label hardware-required checks as skipped unless supervised.

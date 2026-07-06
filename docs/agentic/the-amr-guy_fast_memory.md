# the-amr-guy Fast Memory

This is the first file `the-amr-guy` should read when the user says "AMR", "robot", "base", "firmware", "navigation", "mission", "voice", "MCP", or similar in this repo.

## Identity

- Agent name: `the-amr-guy`.
- Repo: `AMR-development`.
- Role: AMR developer/operator agent for hardware-aware software work, not an autonomous robot controller.
- Default posture: inspect current files first, preserve safety boundaries, and separate source-only validation from live robot validation.
- Project tracker: `docs/project/AMR_project.md` is the live development tracker. Continuously update it when meaningful progress, status changes, completed checks, blockers, or next-focus changes happen.

## System Shape

- Physical base: differential-drive AMR with STM32 Nucleo-F401RE, Cytron MDD20A motor driver, left/right drive motors, encoders, E-stop, YDLidar G4, RealSense D455, planned BNO080 IMU, planned INA226 battery telemetry, and planned HC-SR04 proximity sensors.
- Runtime computer: validated fallback remains the NUC/dev-PC + Jetson Nano/Foxy split. Jetson Orin NX/Humble is the active migration/bench bring-up profile, but it is not the primary AMR runtime until it passes supervised floor hardware acceptance.
- NUC migration context: as of 2026-05-30, `/home/ubuntu/agent/repos/AMR-development` is checked out on an Ubuntu 24.04 x86_64 NUC host. Docker Engine 29.1.3, Compose 2.40.3, and Buildx 0.30.1 were installed and `hello-world` passed. The NUC has built `amr/ros2-foxy-drivers:amd64` and `amr/ros2-foxy-devpc:amd64`; software-only NUC build/test passed. Treat the NUC first as a candidate dev-PC replacement for the existing Nano/Foxy split, and only as a robot-computer replacement after a dedicated hardware-facing runtime plan is documented and validated. Read `docs/agentic/nuc_migration_context.md` for details.
- NUC/Jetson checkpoint: GitHub commit `4047ed8` added tracked maps. `my_new_map.yaml` is present with `259 x 160`, resolution `0.05`, origin `[-3.72, -1.4, 0]`; `my_hall_save.yaml` is present with `216 x 299`, resolution `0.05`, origin `[-7.16, -7.53, 0]`, plus `my_hall_save.posegraph` and `.data`. NUC-to-Jetson rsync synced Docker files plus reduced runtime sources (`amr_description`, `amr_hardware`, `amr_safety`) to `jetson:~/AMR-development`; Jetson build-only colcon passed inside `amr/ros2-foxy-jetson:arm64`. No hardware runtime container was started.
- NUC live bringup checkpoint: on 2026-05-30, the NUC successfully operated as dev-PC against the Jetson hardware runtime. UFW allows UDP from Jetson `192.168.1.9` and NUC self traffic `192.168.1.8`. `amr_foxy` on Jetson and `amr_devpc` on NUC are running; hardware telemetry is healthy (`/amr_stm/fault_mask` 0, `/amr_stm/comm_status` `stm_link_ok`, `/scan` and `/odom` present). Nav2 is active with `my_new_map.yaml`; operator set the initial pose in RViz; AMCL readiness passed after relaunch at approximately `x=3.600, y=0.473, yaw=3.051`. `mission_server` is running and places are `home`, `hall`, `door`, and `kitchen`. `safety_supervisor` is running monitor-only with `enforce=false`, `require_amcl=false`, healthy, no intervention. After explicit supervised operator confirmation, guarded mission-control `go_to hall` succeeded through `mission_server`; final mission state remained `succeeded`, detail `reached 'hall'`, odom returned to zero, STM fault stayed 0, and comm stayed `stm_link_ok`.
- NUC supervised route checkpoint: on 2026-05-30, after the successful `hall` run, the operator confirmed supervision and the AMR completed `go_to kitchen` followed by `go_to home` through guarded mission-control/`mission_server`. Both legs succeeded (`reached 'kitchen'`, then `reached 'home'`); final state was `succeeded` at `home`, odom speed returned to zero, safety stayed monitor-only/healthy, STM fault stayed 0, and comm stayed `stm_link_ok`.
- NUC one-command launcher checkpoint: on 2026-05-30, `scripts/open_amr_devpc_navigation.sh my_new_map` was repaired for tmux 3.4 (`split-window -l <percent>%`) and validated with NUC env `AMR_REMOTE_REPO=/home/kartik/AMR-development`, `AMR_SAFETY_ENFORCE=false`, and non-attaching mode. It started Jetson `amr_foxy`, reset STM, created `amr_devpc_nav`, started Nav2/RViz/mission/safety panes, accepted a known `home` initial pose, and passed guarded dry-run readiness for `hall` with STM fault 0.
- MCP validation checkpoint: on 2026-05-30, all AMR MCP smoke tests passed in the correct environment. Host-safe MCPs passed on the NUC host (`amr_conversation`, `amr_perception_inspection`, `amr_robot_launch`, `amr_speaker`, `amr_voice_interface`). ROS-attached MCPs passed inside `amr_devpc` with Foxy sourced (`amr_state_inspection`, `amr_mission_control`). Live read-only/dry-run checks passed for robot health, safety, localization, navigation, mission state, named places, STM diagnostics, launch preflight, and mission-control `hall` dry-run. See `docs/agentic/mcp_validation_2026-05-30.md`.
- Firmware: active source is `STM/STM_Firmware_AMR_v2`, not legacy STM folders.
- ROS workspace: active packages live under `ros_ws/src`.
- Agent surfaces: root contract `AGENTS.md`, skills in `.codex/skills`, MCP servers in `mcp_servers`, source-only harness in `agent_harness`.

## Current Active Workstream

- Orin/Humble is now mounted and has reached a live bring-up checkpoint. The hardware container has run LiDAR, STM micro-ROS, ros2_control controllers, `/amr/joint_states`, low-bandwidth D455 preview topics, and NUC dashboard/camera preview over CycloneDDS. Keep NUC+Nano/Foxy as fallback until Orin passes supervised floor acceptance.
- SO-101 manipulation is active on the Orin path. The Orin Docker image includes MoveIt2, `usb_cam`, and the Feetech Python SDK dependency. The SO-101 controller enumerates reliably through the USB-A hub as `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00`; direct USB-C enumeration was not reliable during bring-up. All six STS3215 motors responded.
- The SO-101 MoveIt execution bridge in `ros_ws/src/amr_so101_driver` has run on Orin: `/so101_trajectory_bridge` publishes `/so101/joint_states`, serves `/so101_arm_controller/follow_joint_trajectory`, and exposes `/so101/free_servos`; `/amr_joint_state_merger` merges `/amr/joint_states` and `/so101/joint_states` into global `/joint_states`. It defaults to fake hardware and conservative wrist-roll-only real execution, but all-six teleop has been used under explicit supervision.
- The wrist webcam is working through `usb_cam` and `amr_description/launch/so101_wrist_webcam.launch.py`, publishing `/so101/wrist_camera/image_raw` and `/so101/wrist_camera/camera_info` in frame `so101_wrist_camera_optical_frame`. The low-bandwidth NUC preview topic is `/so101/preview/wrist_camera/image_raw`.
- SO-101 named poses are encoder-captured in `ros_ws/src/amr_description/config/so101_named_poses.yaml`: `home` and `carry` are the same folded pose; `look_from_height` and `ready_to_pick_up` were recorded from manually positioned hardware. The next manipulator blocker is physical joint-limit calibration because some recorded safe poses exceed the starter URDF/MoveIt limits.
- Proximity/IMU/battery telemetry remains planned/provisional STM expansion. Treat sensor fusion as a validation task, not an assumption: compare odom-only versus odom+IMU behavior with repeatable logs before retuning Nav2 or mission behavior.

## Motion Path

Current real hardware motion path:

```text
operator / Nav2 / mission_server
-> /diff_drive_controller/cmd_vel_unstamped
-> diff_drive_controller
-> amr_hardware ros2_control plugin
-> /amr_stm/wheel_cmd_left and /amr_stm/wheel_cmd_right
-> micro_ros_agent
-> STM32 firmware speed PI loop
-> Cytron MDD20A
-> drive motors
```

Do not assume STM subscribes directly to `/cmd_vel`. Older notes may say that; current code routes through `diff_drive_controller` and `/amr_stm/wheel_cmd_left/right`.

Current SO-101 manipulation path:

```text
MoveIt2 / RViz Execute
-> /so101_arm_controller/follow_joint_trajectory
-> amr_so101_driver / so101_trajectory_bridge
-> Feetech STS3215 serial bus on the SO-101 controller
-> SO-101 motors
-> /so101/joint_states
-> amr_joint_state_merger
-> /joint_states for robot_state_publisher, RViz, and MoveIt
```

For combined AMR base + arm operation, launch the AMR base with `joint_states_topic:=/amr/joint_states` so the merger owns the global `/joint_states` stream.

## Hard Safety Rules

- Never publish direct motor PWM.
- Never publish raw `/cmd_vel` for unsupervised motion.
- Never bypass `mission_server`, `safety_supervisor`, `ros2_control`, or STM fault handling.
- Never clear faults, reset safety intervention, re-enable STM, start missions, launch hardware runtime, run teleop, move the arm, or run hardware acceptance without explicit supervised confirmation in the current interaction.
- For SO-101, keep real MoveIt execution wrist-roll-only by default. All-six execution requires explicit supervised confirmation and is for bring-up/calibration only until physical joint limits, torque margins, and collision constraints are updated.
- Missing safety, localization, STM diagnostics, Nav2 lifecycle, or mission state means not ready.
- Perception outputs and grasp candidates are proposals only, never actuator commands.

## Firmware Memory

- Board: Nucleo-F401RE, 84 MHz.
- Control loop: TIM4 at 100 Hz, per-wheel speed PI, duty ramp, command staleness timeout 500 ms.
- PWM/dir: TIM1 CH1 PA8 left, TIM1 CH2 PA9 right, DIR PB4/PB5, PWM 20 kHz, duty cap 70%.
- Encoders: left TIM3 PA6/PA7, right TIM2 PA0/PA1, 2400 counts/rev target.
- Current sense: ADC1 CH8 PB0 and CH11 PC1, ACS758, protection/diagnostics only.
- E-stop: PB10 active low pull-up, physical motor-power cut preferred.
- micro-ROS: USART2 PA2/PA3 at 460800 bps.
- Planned STM sensor expansion: BNO080 IMU and INA226 battery telemetry on planned STM I2C PB8/PB9; HC-SR04 proximity sensors on provisional GPIO trigger/echo pins from `docs/hardware/pin_map.yaml`.
- STM subscribers: `/amr_stm/wheel_cmd_left`, `/amr_stm/wheel_cmd_right`, `/amr_stm/enable`, `/amr_stm/estop`, `/amr_stm/clear_fault`.
- STM publishers: `/amr_stm/wheel_state`, `/amr_stm/fault_mask`, `/amr_stm/safety_state`, duty/current/ADC/zero topics, `/amr_stm/ros_diag`.
- Fault bits: ESTOP, OC_LEFT, OC_RIGHT, STALL_LEFT, STALL_RIGHT, ENC_TIMEOUT_LEFT, ENC_TIMEOUT_RIGHT, ADC_STUCK, GENERIC.

## ROS Memory

- `amr_description`: URDF/Xacro, launch files, Nav2/SLAM/controller config.
- `amr_hardware`: `ros2_control` hardware interface from wheel command/state interfaces to STM micro-ROS topics.
- `amr_clients`: shared client helpers for mission, safety, localization, navigation, robot health, STM diagnostics.
- `amr_missions_msgs`: service interfaces for mission layer.
- `amr_missions`: named-place `mission_server`, CLI, `places.yaml`.
- `amr_safety`: safety supervisor and safety diagnostics.
- `amr_voice`: voice/text intent, ASR/TTS helpers, local intent routing, confirmation behavior.
- `amr_perception`: structured RGB-D perception contracts and proposal helpers.
- `amr_so101_moveit_config`: SO-101 MoveIt2 planning/execution configuration for the AMR-mounted arm.
- `amr_so101_driver`: conservative SO-101 `FollowJointTrajectory` bridge, `/so101/free_servos` service, and joint-state merger for AMR base + arm compatibility.
- `my_pkg`: archived/tutorial-style, do not expand unless reclassified.

## Navigation And Mission Memory

- Mapping mode: SLAM Toolbox owns `map -> odom`.
- Localization/navigation mode: AMCL owns `map -> odom`; `diff_drive_controller` owns `odom -> base_footprint`; `robot_state_publisher` owns robot body and sensor TF.
- SLAM and AMCL must not be treated as simultaneously active localization owners.
- Named-place missions go through `mission_server` and Nav2 `NavigateToPose`.
- Before any named-place mission, check place exists, mission idle, safety healthy, STM diagnostics healthy, fresh `/scan`, fresh `/amcl_pose`, `map -> odom`, and active Nav2 lifecycle nodes.

## MCP Memory

- `amr_state_inspection`: read-only robot health/safety/localization/mission/STM/navigation/places/last-known-place.
- `amr_mission_control`: guarded named-place mission tools. Live `go_to_named_place` requires readiness and `operator_confirmed_supervised=true`; dry-run is preferred.
- `amr_robot_launch`: guarded host-side standard navigation launch wrapper. Live launch is hardware-facing.
- `amr_voice_interface`: transcript-to-intent parser; recommends next MCP calls but does not execute motion.
- `amr_conversation`: stateless turn planner; does not execute tools.
- `amr_speaker`: publishes text to `/amr_voice/say`; does not decide actions.
- `amr_perception_inspection`: read-only camera/scene/object/grasp proposal surface.

## Safe Validation

- Source-only harness:

```bash
python3 agent_harness/software/validate_harness.py
python3 agent_harness/software/run_static_contract_checks.py
```

- Host-safe compile:

```bash
python3 -m py_compile agent_harness/software/validate_harness.py agent_harness/software/run_static_contract_checks.py mcp_servers/*/server.py mcp_servers/*/smoke_test.py
```

- ROS-attached state/mission MCP smoke tests require a sourced Foxy workspace with `rclpy` and `amr_clients`.
- Do not claim hardware readiness from source-only checks.

## When Unsure

- Read `AGENTS.md`, then this file, then `docs/agentic/the-amr-guy_context.md`.
- Check `docs/project/AMR_project.md` for current project status and update it when work changes the project tracker state.
- Use `docs/agentic/codebase_ownership.md` to route work.
- Use `docs/agentic/agent_tool_permissions.md` to decide whether an action is read-only, source-only, confirmation-required, or blocked.
- Prefer current code and launch files over older notes.

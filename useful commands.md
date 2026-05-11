# Useful Commands

This file is ordered by day-to-day usefulness. Prefer the one-command launchers first; use the manual sections only when debugging.

## 1. Navigation Bringup

Start the full navigation workflow from the laptop/dev PC:

```bash
cd ~/AMR-development
AMR_SAFETY_ENFORCE=true ./scripts/open_amr_devpc_navigation.sh my_new_map
```

Start the full robot workflow with microphone ASR enabled:

```bash
cd ~/AMR-development
./scripts/open_amr_devpc_full.sh my_new_map
```

This starts:
- Jetson hardware stack in `amr_foxy`
- STM reset over ST-LINK/OpenOCD
- dev-PC container `amr_devpc`
- RViz with `ros_ws/src/amr_description/config/amr.rviz`
- Nav2 localization/navigation
- mission server/status panes
- safety supervisor/status panes
- teleop fallback pane
- voice command pane: typed text by default, microphone ASR in the full launcher

After RViz opens:
- Set the AMR pose with `2D Pose Estimate`.
- Wait for the Nav2 pane to report that AMCL localization is ready.
- Then use mission commands or voice commands.

Map arguments:

```bash
./scripts/open_amr_devpc_navigation.sh my_new_map
./scripts/open_amr_devpc_navigation.sh my_new_map.yaml
./scripts/open_amr_devpc_navigation.sh /workspaces/AMR-development/ros_ws/maps/my_new_map.yaml
```

Use a different RViz config for the one-shot navigation launcher:

```bash
AMR_RVIZ_CONFIG_PATH=/workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz \
  ./scripts/open_amr_devpc_navigation.sh my_new_map
```

The navigation launcher no longer starts legacy voice panes. Voice development is MCP-based:

```bash
AMR_VOICE_MODE=off ./scripts/open_amr_devpc_navigation.sh my_new_map
```

The launcher creates the navigation and monitor tmux windows. RViz starts as a GUI from the Nav2 pane, not as a separate tmux window; RViz logs are written inside the container at `/tmp/amr_rviz.log`.

Window `0:navigation` is selected after launch:
- `Nav2 + AMCL`: launches Nav2, AMCL, and waits for fresh localization before enabling missions.
- `Mission Shell`: interactive shell for `mission_cli`; this pane is selected after launch.
- `Teleop`: keyboard fallback driving on `/diff_drive_controller/cmd_vel_unstamped`.

Window `1:monitor`:
- `Topics`: ROS graph topic list with publisher and subscriber counts. It refreshes every 3 seconds by default.
- `Nodes`: ROS graph node list with publisher and subscriber counts. It refreshes every 3 seconds by default.
- `Mission Server`: builds mission packages and runs `mission_server`.
- `Mission Status`: clean live mission summary with elapsed time and warnings for navigation that looks stuck, idle, or stale.
- `Safety`: runs `safety_supervisor`.
- `Safety Status`: clean live summary of safety health, intervention reasons, STM fault bits, comm state, stale topics, and odom speed.

Tune graph monitor refresh rate if needed:

```bash
AMR_GRAPH_MONITOR_PERIOD=5.0 ./scripts/open_amr_devpc_full.sh my_new_map
```

The topic/node counts come from ROS graph metadata, not from `topic echo` or `topic hz`, so the default refresh is lightweight.

Switch panes by clicking, or use `Ctrl-b` then an arrow key. Switch windows with `Ctrl-b n`, `Ctrl-b p`, or `Ctrl-b 0/1/2`.

For direct ROS launch usage, RViz is also available from the Nav2 navigation launch file:

```bash
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/nav2_navigation.launch.py \
  use_rviz:=true \
  rviz_config:=/workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz
```

## 2. Mission Commands

Run these inside `amr_devpc`, or through `docker exec` from the laptop.

Open a ROS shell:

```bash
docker exec -it amr_devpc /entrypoint.sh bash
cd /workspaces/AMR-development/ros_ws
source install/setup.bash
```

Mission status:

```bash
ros2 run amr_missions mission_cli status
```

List named places:

```bash
ros2 run amr_missions mission_cli list
```

Go to a named place:

```bash
ros2 run amr_missions mission_cli go_to kitchen
ros2 run amr_missions mission_cli go_to hall
ros2 run amr_missions mission_cli go_to home
```

Patrol:

```bash
ros2 run amr_missions mission_cli patrol home hall door --return-home home
```

Cancel active mission:

```bash
ros2 run amr_missions mission_cli cancel
```

Watch structured mission status:

```bash
ros2 topic echo /amr_missions/status
```

Watch wake-word feedback text, which is also a future TTS input:

```bash
ros2 topic echo /amr_voice/feedback
```

Run the MCP voice intent smoke test:

```bash
python3 mcp_servers/amr_voice_interface/smoke_test.py
```

Run the wake-word detector:

```bash
ros2 run amr_voice wake_word_node --model hey_jarvis --threshold 0.5 --dry-run --log-audio-level
```

Transcribe a WAV file with local `whisper.cpp` and emit voice-MCP arguments:

```bash
./scripts/setup_whisper_cpp.sh

# For use from the Foxy container:
AMR_WHISPER_BUILD_DIR=/workspaces/AMR-development/models/whisper.cpp/build-foxy \
  ./scripts/setup_whisper_cpp.sh

ros2 run amr_voice asr_file_cli input.wav \
  --whisper-bin models/whisper.cpp/build-foxy/bin/whisper-cli \
  --model /workspaces/AMR-development/models/whisper/ggml-base.en.bin
```

`asr_file_cli` assumes the wake word was already detected, so the MCP payload does not
require the wake phrase in the transcript text unless `--require-wake-word` is passed.

Run the live wake -> VAD -> ASR dry-run pipeline:

```bash
ros2 run amr_voice voice_pipeline_node \
  --device 9 \
  --whisper-bin /workspaces/AMR-development/models/whisper.cpp/build-foxy/bin/whisper-cli \
  --whisper-model /workspaces/AMR-development/models/whisper/ggml-base.en.bin \
  --log-audio-level
```

Bypass wake detection while tuning VAD/ASR:

```bash
ros2 run amr_voice voice_pipeline_node --start-listening --device 9 ...
```

The removed legacy nodes were:
- `voice_text_cli`
- `voice_command_node`
- `voice_asr_node`
- `scripts/open_amr_voice_asr.sh`

MCP transcript flow:

```text
hey jarvis -> wake event -> VAD/ASR transcript -> amr_voice_interface MCP -> mission-control MCP
```

Current named places:
- `home`
- `door`
- `kitchen`
- `hall`

## 3. Safety Recovery

Use this after a safety intervention or STM fault, once the physical fault source is fixed or ready to be inspected:

```bash
docker exec -it amr_devpc /entrypoint.sh bash -lc '
cd /workspaces/AMR-development
source ros_ws/install/setup.bash
python3 scripts/amr_safety_recover.py
'
```

The helper:
- cancels mission/Nav2 motion
- publishes zero velocity
- disables STM motor output
- prints decoded STM/comm/supervisor state
- prompts before clearing a nonzero STM fault
- calls `/amr/safety_supervisor/reset_intervention`
- asks separately before re-enabling STM

Scripted mode, only after the physical fault source is definitely fixed:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc '
cd /workspaces/AMR-development
source ros_ws/install/setup.bash
python3 scripts/amr_safety_recover.py --assume-fixed --reenable --no-prompt-reenable
'
```

Manual fallback:

```bash
ros2 run amr_missions mission_cli cancel
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}, stamp: {sec: 0, nanosec: 0}}}"
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{}"
ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /amr_stm/clear_fault std_msgs/msg/Empty "{}"
ros2 service call /amr/safety_supervisor/reset_intervention std_srvs/srv/Trigger "{}"
ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: true}"
```

## 4. Health Checks

Short baseline probe:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc '
cd /workspaces/AMR-development
source ros_ws/install/setup.bash
python3 scripts/amr_baseline_probe.py --duration 30
'
```

Quick graph checks:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc '
source /workspaces/AMR-development/ros_ws/install/setup.bash
ros2 node list | sort
ros2 control list_controllers
ros2 action info /navigate_to_pose
ros2 topic info /amr_stm/wheel_state
ros2 topic info /amr/safety_supervisor/status
'
```

Decode a fault mask:

```bash
python3 scripts/amr_decode_faults.py --fault-mask 24
python3 scripts/amr_decode_faults.py --safety-state 65560
python3 scripts/amr_decode_faults.py --comm-fault-mask 2
```

ROS daemon refresh when discovery looks stale:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc '
source /workspaces/AMR-development/ros_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
ros2 node list
'
```

## 5. AMR Monitor

Open the live monitor from the laptop/dev PC:

```bash
cd ~/AMR-development
./scripts/open_amr_monitor.sh
```

If the layout is stale or broken:

```bash
tmux kill-session -t amr_bench 2>/dev/null || true
cd ~/AMR-development
./scripts/open_amr_monitor.sh
```

Monitor layout:
- left column: launch/status
- middle column: node and safety/fault state
- right column: wheel summaries, agent log, and command shell

## 6. Teleop

Teleop from the laptop Docker container:

```bash
docker exec -it amr_devpc /entrypoint.sh bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /workspaces/AMR-development/ros_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped \
  -p speed:=0.1 \
  -p turn:=0.15
```

Repo teleop fallback:

```bash
docker exec -it amr_devpc /entrypoint.sh bash -lc '
source /workspaces/AMR-development/ros_ws/install/setup.bash
python3 /workspaces/AMR-development/scripts/amr_teleop_keyboard.py \
  --speed 0.1 \
  --turn 0.15 \
  --topic /diff_drive_controller/cmd_vel_unstamped
'
```

## 7. SLAM And Map Save

Start SLAM workflow:

```bash
cd ~/AMR-development
./scripts/open_amr_devpc_slam.sh
```

For a full-house map, set the output prefix before launch so the map-save pane prints the right command:

```bash
cd ~/AMR-development
AMR_MAP_SAVE_PREFIX=/workspaces/AMR-development/ros_ws/maps/full_house_map \
  ./scripts/open_amr_devpc_slam.sh
```

Drive slowly with the teleop pane. Revisit already-mapped areas to close loops before saving.

Save map from inside `amr_devpc` or from the map-save pane:

```bash
docker exec -it amr_devpc /entrypoint.sh bash
source /workspaces/AMR-development/ros_ws/install/setup.bash

ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f /workspaces/AMR-development/ros_ws/maps/full_house_map \
  --ros-args -p save_map_timeout:=10000
```

After saving the map, tag house areas by adding/updating entries in:

```text
ros_ws/src/amr_missions/config/places.yaml
```

Use RViz `Publish Point` or the robot pose in RViz to choose `x`, `y`, and `yaw` values in the `map` frame, then validate each tag with:

```bash
ros2 run amr_missions mission_cli list
ros2 run amr_missions mission_cli go_to <place_name>
```

## 8. Hardware-Only Bringup

Use this when validating STM, motors, odom, and LiDAR without Nav2.

```bash
ssh kartik@192.168.1.9 bash -s <<'EOF'
set -euo pipefail

container_name="amr_foxy"
remote_repo="$HOME/AMR-development"
agent_dev="/dev/ttyACM0"
agent_baud="460800"

plugdev_gid="$(getent group plugdev | cut -d: -f3 || true)"
docker_group_args=()
if [[ -n "${plugdev_gid}" ]]; then
  docker_group_args+=(--group-add "${plugdev_gid}")
fi

docker rm -f "${container_name}" >/dev/null 2>&1 || true

docker run -d --name "${container_name}" --net=host --privileged --runtime nvidia \
  "${docker_group_args[@]}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "${remote_repo}/ros_ws:/workspaces/ros_ws" \
  amr/ros2-foxy-jetson:arm64 \
  bash -lc "
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /workspaces/ros_ws
[ -f /opt/ros/driver_ws/install/setup.bash ] && source /opt/ros/driver_ws/install/setup.bash
colcon build --merge-install --symlink-install --packages-select amr_hardware amr_description
source install/setup.bash
ros2 launch amr_description hardware.launch.py \
  use_sim_time:=false \
  agent_dev:=${agent_dev} \
  agent_baud:=${agent_baud} \
  start_lidar:=true \
  start_camera:=false \
  start_link_watchdog:=false
" >/dev/null
EOF
```

Verify:

```bash
ssh kartik@192.168.1.9 "docker exec amr_foxy /entrypoint.sh bash -lc '
source /workspaces/ros_ws/install/setup.bash
ros2 node list | sort
ros2 control list_controllers
ros2 topic info /amr_stm/wheel_state
timeout 5s ros2 topic echo /amr_stm/fault_mask std_msgs/msg/Int32 --qos-reliability best_effort
'"
```

Pass condition:
- `/amr_firmware` is present.
- `joint_state_broadcaster` and `diff_drive_controller` are active.
- `/amr_stm/wheel_state` has `Publisher count: 1`.
- `/amr_stm/fault_mask` stays at `0` while idle.

## 9. Manual Dev PC Container

Use only when the one-command launchers are not appropriate.

```bash
pkill -f rviz2 || true
xhost +local:root
docker rm -f amr_devpc 2>/dev/null || true

docker run -it --name amr_devpc --net=host \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-foxy-devpc:amd64 \
  /entrypoint.sh bash
```

## 10. Manual Nav2 Launch Fallbacks

Use these only when debugging launch files directly.

RViz:

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
  -d /workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz
```

Localization only:

```bash
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=false \
  map:=/workspaces/AMR-development/ros_ws/maps/my_new_map.yaml \
  params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/nav2_params_amr.yaml
```

Navigation after AMCL is localized:

```bash
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/nav2_navigation.launch.py \
  use_sim_time:=false
```

Combined bringup when initial pose is already valid:

```bash
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/bringup_nav2.launch.py \
  use_sim_time:=false \
  map:=/workspaces/AMR-development/ros_ws/maps/my_new_map.yaml \
  start_navigation:=true
```

Important:
- Do not run slam_toolbox while running Nav2 localization/navigation.
- Do not start another `robot_state_publisher` on the dev PC; `hardware.launch.py` starts it on the Jetson.

## 11. Build And Sync

Sync Docker files to Jetson:

```bash
rsync -av --delete --exclude .git \
  /home/kartik/AMR-development/docker/ \
  kartik@192.168.1.9:~/AMR-development/docker/
```

Build Jetson images from dev PC:

```bash
ssh -t jetson '
cd ~/AMR-development &&
docker buildx build -f docker/foxy/Dockerfile \
  --platform linux/arm64 \
  --build-arg BASE_IMAGE=dustynv/ros:foxy-ros-base-l4t-r32.7.1 \
  -t amr/ros2-foxy-drivers:arm64 --load . &&
docker buildx build -f docker/foxy/Dockerfile.jetson \
  --build-arg DRIVER_IMAGE=amr/ros2-foxy-drivers:arm64 \
  -t amr/ros2-foxy-jetson:arm64 --load .
'
```

Build dev-PC image:

```bash
cd ~/AMR-development
docker build -f docker/foxy/Dockerfile.devpc \
  -t amr/ros2-foxy-devpc:amd64 .
```

Rebuild selected packages inside `amr_devpc`:

```bash
docker exec -it amr_devpc /entrypoint.sh bash -lc '
cd /workspaces/AMR-development/ros_ws
source /opt/ros/foxy/setup.bash
colcon build --merge-install --symlink-install --packages-select amr_missions_msgs amr_missions amr_safety
source install/setup.bash
'
```

Rebuild `amr_description` in running Jetson container:

```bash
ssh jetson "docker exec -it amr_foxy bash -lc '
source /opt/ros/foxy/install/setup.bash
[ -f /opt/ros/driver_ws/install/setup.bash ] && source /opt/ros/driver_ws/install/setup.bash
cd /workspaces/ros_ws
colcon build --merge-install --symlink-install --packages-select amr_description
source install/setup.bash
'"
```

## 12. STM And Jetson Handshake

STM serial access note:
- On this Jetson, `/dev/ttyACM0` is owned by `root:plugdev`, not `dialout`.
- Jetson-side `amr_foxy` containers that need the STM port must include:
  `--group-add "$(getent group plugdev | cut -d: -f3)"`.
- The one-command launchers already include this.

Validate that `micro_ros_agent` can open the STM port:

```bash
PLUGDEV_GID="$(getent group plugdev | cut -d: -f3)"
docker run --rm -it --net=host --privileged --runtime nvidia \
  --group-add "${PLUGDEV_GID}" \
  --name amr_foxy \
  amr/ros2-foxy-jetson:arm64 \
  /entrypoint.sh bash -lc "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 460800 -v 6"
```

After STM reset or power-cycle, confirm the first required publisher exists:

```bash
docker exec -it amr_foxy /entrypoint.sh bash -lc "ros2 topic info -v /amr_stm/wheel_state"
```

Validate base readiness:

```bash
docker exec -it amr_foxy /entrypoint.sh bash -lc "ros2 control list_controllers"
docker exec -it amr_foxy /entrypoint.sh bash -lc "timeout 5s ros2 topic echo /amr_stm/wheel_state sensor_msgs/msg/JointState --qos-reliability best_effort"
docker exec -it amr_foxy /entrypoint.sh bash -lc "timeout 5s ros2 run tf2_ros tf2_echo odom base_footprint"
```

## 13. STM Topic Inventory

STM firmware subscribes to:
- `/amr_stm/wheel_cmd_left`
- `/amr_stm/wheel_cmd_right`
- `/amr_stm/enable`
- `/amr_stm/estop`
- `/amr_stm/clear_fault`

STM firmware publishes:
- `/amr_stm/wheel_state`
- `/amr_stm/fault_mask`
- `/amr_stm/safety_state`
- `/amr_stm/duty_cmd_left`
- `/amr_stm/duty_cmd_right`
- `/amr_stm/current_left_ma`
- `/amr_stm/current_right_ma`
- `/amr_stm/current_left_adc`
- `/amr_stm/current_right_adc`
- `/amr_stm/current_left_zero`
- `/amr_stm/current_right_zero`

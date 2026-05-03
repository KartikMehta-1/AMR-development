# Useful Commands

This file is ordered by day-to-day usefulness. Prefer the one-command launchers first; use the manual sections only when debugging.

## 1. Navigation Bringup

Start the full navigation workflow from the laptop/dev PC:

```bash
cd ~/AMR-development
AMR_SAFETY_ENFORCE=true ./scripts/open_amr_devpc_navigation.sh my_new_map
```

This starts:
- Jetson hardware stack in `amr_foxy`
- STM reset over ST-LINK/OpenOCD
- dev-PC container `amr_devpc`
- RViz
- Nav2 localization/navigation
- mission server/status panes
- safety supervisor/status panes
- teleop fallback pane

After RViz opens:
- Set the AMR pose with `2D Pose Estimate`.
- Wait for the Nav2 pane to report that AMCL localization is ready.
- Then use mission commands.

Map arguments:

```bash
./scripts/open_amr_devpc_navigation.sh my_new_map
./scripts/open_amr_devpc_navigation.sh my_new_map.yaml
./scripts/open_amr_devpc_navigation.sh /workspaces/AMR-development/ros_ws/maps/my_new_map.yaml
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

Text command interface:

```bash
ros2 run amr_voice voice_text_cli
```

Wake-gated text command interface:

```bash
ros2 run amr_voice voice_text_cli --wake-gated
```

Example typed commands:

```text
lovely go kitchen
lovely
go hall
return home
stop
status
list places
```

In wake-gated mode, `lovely` opens a short listening window for the next command. `stop` / `cancel` is still accepted without the wake word.
Motion commands now require confirmation by default. After `lovely go kitchen`, say or type `yes` to start the mission, or `no` to discard it. Use `--no-confirm-motion` only for controlled testing.

One-shot dry run, useful before enabling motion:

```bash
ros2 run amr_voice voice_text_cli --wake-gated --dry-run --command "lovely go to the kitchen"
```

Laptop microphone ASR setup:

```bash
mkdir -p /workspaces/AMR-development/models
cd /workspaces/AMR-development/models
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
```

List microphone devices:

```bash
ros2 run amr_voice voice_asr_node --list-devices
```

Run ASR in dry-run mode:

```bash
ros2 run amr_voice voice_asr_node --dry-run --device 9
```

Run ASR against mission commands:

```bash
ros2 run amr_voice voice_asr_node --device 9
```

If recognition becomes poor after a container or laptop audio restart, rerun `ros2 run amr_voice voice_asr_node --list-devices`. Use the digital mic input-only device with a 16 kHz default rate (`hw:1,7`, currently index `9` on this laptop); avoid HDMI output-only devices and the silent headset/analog input path.

ASR motion-command flow:

```text
lovely
go to kitchen
yes
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

Save map from inside `amr_devpc`:

```bash
docker exec -it amr_devpc /entrypoint.sh bash
source /workspaces/AMR-development/ros_ws/install/setup.bash

ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f /workspaces/AMR-development/ros_ws/maps/my_new_map \
  --ros-args -p save_map_timeout:=10000
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

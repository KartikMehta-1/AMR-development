# Useful Commands

## Quick Start (Top 4)

STM serial-access note:
- On this Jetson, `/dev/ttyACM0` is owned by `root:plugdev`, not `dialout`.
- Any Jetson-side `amr_foxy` container that needs the STM port must include the host `plugdev` group with `--group-add "$(getent group plugdev | cut -d: -f3)"`.
- The repo scripts already patched for this are:
  - `scripts/open_amr_devpc_navigation.sh`
  - `scripts/open_amr_devpc_slam.sh`
  - `scripts/open_amr_devpc_localization.sh`

### One-command AMR monitor (from dev PC)
This runs the bench monitor from the desktop over SSH. It opens a local tmux session when `tmux` is installed, otherwise it falls back to terminal tabs. On the Jetson side it reuses `amr_foxy` if it is already running, or starts an agent-only container if needed. The monitor watches the STM firmware namespace `/amr_stm/*`.

Layout:
- left column: launch status
- middle column: node status over safety/fault state
- right column: wheel summaries, agent log, and command shell


If the layout gets stale or broken, recreate it:
```bash
tmux kill-session -t amr_bench 2>/dev/null || true
cd ~/AMR-development
./scripts/open_amr_monitor.sh
```

### 1) Jetson: AMR hardware-only bringup (motors + lidar, no Nav2)
Run from the dev PC:
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

### 2) Dev PC: One-command SLAM launcher
This starts:
- the Jetson hardware stack (`amr_foxy`)
- Jetson ST-LINK reset of the STM after startup
- the `amr_devpc` container
- RViz
- `slam_toolbox`
- keyboard teleop

```bash
cd ~/AMR-development
./scripts/open_amr_devpc_slam.sh
```

Save the map from inside the `amr_devpc` container once SLAM looks good:

```bash
docker exec -it amr_devpc /entrypoint.sh bash
ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f /workspaces/AMR-development/ros_ws/maps/my_new_map \
  --ros-args -p save_map_timeout:=10000
```

### 3) Dev PC: One-command navigation launcher
This starts:
- the Jetson hardware stack (`amr_foxy`)
- Jetson ST-LINK reset of the STM after startup
- the `amr_devpc` container
- RViz
- Nav2 localization on a saved map
- Nav2 navigation after AMCL has a valid initial pose
- `mission_server`
- live mission status pane
- mission command shell
- keyboard teleop for fallback checks

```bash
cd ~/AMR-development
./scripts/open_amr_devpc_navigation.sh my_new_map
```

Important startup step:
- After RViz opens, use `2D Pose Estimate` to set the AMR pose on the map.
- The Nav2 pane waits for AMCL to publish `map -> odom`; only then does it start planner/controller navigation.
- Do not send a mission until the Nav2 pane prints `AMCL pose is active; starting Nav2 navigation lifecycle.`

You can pass:
- `my_new_map`
- `my_new_map.yaml`
- `/workspaces/AMR-development/ros_ws/maps/my_new_map.yaml`

### 4) Mission layer: persistent runtime (inside `amr_devpc`)
Build and source the mission packages:

```bash
docker exec -it amr_devpc /entrypoint.sh bash
cd /workspaces/AMR-development/ros_ws
colcon build --merge-install --packages-select amr_missions_msgs amr_missions
source install/setup.bash
```

Start the mission server:

```bash
ros2 run amr_missions mission_server
```

Query mission runtime state:

```bash
ros2 run amr_missions mission_cli status
ros2 service call /amr_missions/state amr_missions_msgs/srv/GetMissionState "{}"
```

List named places:

```bash
ros2 run amr_missions mission_cli list
```

Go to a named place:

```bash
ros2 run amr_missions mission_cli go_to kitchen
ros2 run amr_missions mission_cli go_to hall
```

Run a simple patrol and return home:

```bash
ros2 run amr_missions mission_cli patrol home hall door --return-home home
```

Cancel the active mission:

```bash
ros2 run amr_missions mission_cli cancel
```

Watch structured mission status:

```bash
ros2 topic echo /amr_missions/status
```

Topic-command interface:

```bash
ros2 topic pub --once /amr_missions/command std_msgs/msg/String "{data: 'go_to:kitchen'}"
ros2 topic pub --once /amr_missions/command std_msgs/msg/String "{data: 'patrol:home,hall,door'}"
ros2 topic pub --once /amr_missions/command std_msgs/msg/String "{data: 'cancel'}"
```

Current named places:
- `home`
- `door`
- `kitchen`
- `hall`

### 5) Dev PC: Manual container + tool commands (fallback)
Use this on the laptop when you want a ROS 2 shell that can see the Jetson ROS graph.

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

### 6) Dev PC: Teleop from laptop Docker
Start `amr_devpc` with the previous section, then run:

```bash
docker exec -it amr_devpc /entrypoint.sh bash
unset CYCLONEDDS_URI
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped \
  -p speed:=0.1 \
  -p turn:=0.15
```

### 7) Dev PC: Launch/Run SLAM toolbox, RViz, AMCL, Nav2 (inside amr_devpc container)

```bash
docker exec -it amr_devpc /entrypoint.sh bash

# RViz (software rendering fallback)
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d /workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz

# SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/slam_toolbox_online_async.yaml

```

Important:
- Do not start `robot_state_publisher` again on the dev PC.
- `hardware.launch.py` already starts `robot_state_publisher` on the Jetson, which is the correct place for it in this stack.



### 7) Load saved map (Nav2 AMCL localization)
Use this after you already have a saved `*.yaml` + `*.pgm` in `ros_ws/maps/`.

Important: don't run AMCL at the same time as slam_toolbox mapping/localization (both publish `map->odom`).

```bash
# Start map_server + AMCL (publishes /map and the map->odom TF once localized)
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=false \
  map:=/workspaces/AMR-development/ros_ws/maps/my_hall_save.yaml \
  params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/nav2_params_amr.yaml

# In RViz: use "2D Pose Estimate" to set the initial pose on the map.
```

### 8) Nav2 navigation (AMCL first, then Planner/Controller)
Start AMCL localization first. Start the planner/controller servers only after RViz `2D Pose Estimate` has made `map -> odom` available.

Important: don't run slam_toolbox while running Nav2 localization/navigation (both publish `map->odom`).

```bash
# Option A (recommended): one launch file, localization only by default.
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/bringup_nav2.launch.py \
  use_sim_time:=false \
  map:=/workspaces/AMR-development/ros_ws/maps/my_new_map.yaml

# In RViz, set initial pose with "2D Pose Estimate", then start navigation:
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/nav2_navigation.launch.py \
  use_sim_time:=false

# Option B: only if you already have a valid initial pose and map->odom is present,
# bring up localization and navigation together:
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/bringup_nav2.launch.py \
  use_sim_time:=false \
  map:=/workspaces/AMR-development/ros_ws/maps/my_new_map.yaml \
  start_navigation:=true

# Option C: if AMCL is already running, start only the navigation servers.
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/nav2_navigation.launch.py \
  use_sim_time:=false
```

## Build & Sync
### Sync docker folder to Jetson
```bash
rsync -av --delete --exclude .git /home/kartik/AMR-development/docker/ kartik@192.168.1.9:~/AMR-development/docker/
```

### Build Jetson images from dev PC
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

### Build Dev PC image
```bash
cd ~/AMR-development

docker build -f docker/foxy/Dockerfile.devpc \
  -t amr/ros2-foxy-devpc:amd64 .
```




### Rebuild `amr_description` in running Jetson container
```bash
docker exec -it amr_foxy bash -lc '
source /opt/ros/foxy/install/setup.bash
[ -f /opt/ros/driver_ws/install/setup.bash ] && source /opt/ros/driver_ws/install/setup.bash
cd /workspaces/ros_ws
colcon build --merge-install --symlink-install --packages-select amr_description
source install/setup.bash
'
```

## Diagnostics
```bash
ros2 daemon stop && ros2 daemon start
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 topic list | grep map
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### STM firmware topic inventory
Current STM firmware subscribes to:
- `/amr_stm/wheel_cmd_left`
- `/amr_stm/wheel_cmd_right`
- `/amr_stm/enable`
- `/amr_stm/estop`
- `/amr_stm/clear_fault`

Current STM firmware publishes:
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

### STM and Jetson handshake validation
Use this after changing STM transport behavior or after patching Jetson-side container access.

1. Validate that `micro_ros_agent` can open the STM port:

```bash
PLUGDEV_GID="$(getent group plugdev | cut -d: -f3)"
docker run --rm -it --net=host --privileged --runtime nvidia \
  --group-add "${PLUGDEV_GID}" \
  --name amr_foxy \
  amr/ros2-foxy-jetson:arm64 \
  /entrypoint.sh bash -lc "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 460800 -v 6"
```

Pass condition:
- no immediate `Error while starting serial agent!`

2. After STM reset or power-cycle, confirm the first required publisher exists:

```bash
docker exec -it amr_foxy /entrypoint.sh bash -lc "ros2 topic info -v /amr_stm/wheel_state"
```

Pass condition:
- `Publisher count: 1` or higher

3. Validate full base readiness:

```bash
docker exec -it amr_foxy /entrypoint.sh bash -lc "ros2 control list_controllers"
docker exec -it amr_foxy /entrypoint.sh bash -lc "timeout 5s ros2 topic echo /amr_stm/wheel_state sensor_msgs/msg/JointState --qos-reliability best_effort"
docker exec -it amr_foxy /entrypoint.sh bash -lc "timeout 5s ros2 run tf2_ros tf2_echo odom base_footprint"
```

Pass condition:
- `joint_state_broadcaster` active
- `diff_drive_controller` active
- `/amr_stm/wheel_state` streaming
- `odom -> base_footprint` exists

### AMR fault clear / safe state
```bash
ros2 topic pub --once /amr_stm/enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /amr_stm/estop std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /amr_stm/clear_fault std_msgs/msg/Empty "{}"
```

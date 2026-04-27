# Useful Commands

## Quick Start (Top 4)

### One-command AMR monitor (from dev PC)
This runs the bench monitor from the desktop over SSH. It opens a local tmux session when `tmux` is installed, otherwise it falls back to terminal tabs. On the Jetson side it reuses `amr_foxy` if it is already running, or starts an agent-only container if needed.

Layout:
- left column: launch status
- middle column: left wheel summary over right wheel summary
- right column: safety/fault state over command shell


If the layout gets stale or broken, recreate it:
```bash
tmux kill-session -t amr_bench 2>/dev/null || true
cd ~/AMR-development
./scripts/open_amr_monitor.sh
```

### 1) Jetson: AMR hardware bringup (motors + lidar)
```bash
docker run -d --name amr_foxy --net=host --privileged --runtime nvidia \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -v ~/AMR-development/ros_ws:/workspaces/ros_ws \
  amr/ros2-foxy-jetson:arm64 \
  bash -lc "ros2 launch amr_description hardware.launch.py \
    use_sim_time:=false \
    agent_baud:=460800 \
    start_lidar:=true \
    start_camera:=false"
```

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
docker exec -it amr_devpc bash
source /opt/ros/foxy/setup.bash
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
- full Nav2 bring-up on a saved map
- keyboard teleop for fallback checks

```bash
cd ~/AMR-development
./scripts/open_amr_devpc_navigation.sh my_new_map
```

You can pass:
- `my_new_map`
- `my_new_map.yaml`
- `/workspaces/AMR-development/ros_ws/maps/my_new_map.yaml`

### 4) Mission layer: named-place commands (inside `amr_devpc`)
Build and source the mission package:

```bash
docker exec -it amr_devpc bash
cd /workspaces/AMR-development/ros_ws
source /opt/ros/foxy/setup.bash
colcon build --merge-install --packages-select amr_missions
source install/setup.bash
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

Current named places:
- `home`
- `door`
- `kitchen`
- `hall`

### 5) Dev PC: Manual container + tool commands (fallback)
```bash
pkill -f rviz2 || true
xhost +local:root
docker rm -f amr_devpc 2>/dev/null || true

docker run -it --name amr_devpc --net=host \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-foxy-devpc:amd64 \
  bash
```

### 6) Dev PC: Launch/Run SLAM toolbox, RViz, Teleop, AMCL, Nav2 (inside amr_devpc container)

```bash
docker exec -it amr_devpc bash
source /opt/ros/foxy/setup.bash

# RViz (software rendering fallback)
source /opt/ros/foxy/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d /workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz

# Teleop (slow)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped \
  -p speed:=0.03 -p turn:=0.3

# SLAM Toolbox
source /opt/ros/foxy/setup.bash
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
source /opt/ros/foxy/setup.bash

# Start map_server + AMCL (publishes /map and the map->odom TF once localized)
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=false \
  map:=/workspaces/AMR-development/ros_ws/maps/my_hall_save.yaml \
  params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/nav2_params_amr.yaml

# In RViz: use "2D Pose Estimate" to set the initial pose on the map.
```

### 8) Nav2 navigation (Map + AMCL + Planner/Controller)
This is the full navigation stack (AMCL localization + global planner + local controller).

Important: don't run slam_toolbox while running Nav2 localization/navigation (both publish `map->odom`).

```bash
source /opt/ros/foxy/setup.bash

# Option A (recommended): launch everything (map_server + AMCL + navigation + RViz)
# (launch by file path so you don't need to rebuild the workspace just to pick up the new launch file)
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/bringup_nav2.launch.py \
  use_sim_time:=false \
  map:=/workspaces/AMR-development/ros_ws/maps/my_hall_save.yaml

# Option B: if AMCL is already running, start only the navigation servers (planner/controller)
ros2 launch /workspaces/AMR-development/ros_ws/src/amr_description/launch/nav2_navigation.launch.py \
  use_sim_time:=false

# In RViz:
# - Set initial pose ("2D Pose Estimate")
# - Send goal using the Nav2 Goal tool / Navigation2 panel
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

### AMR fault clear / safe state
```bash
ros2 topic pub --once /amr/enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /amr/estop std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /amr/clear_fault std_msgs/msg/Empty "{}"
```

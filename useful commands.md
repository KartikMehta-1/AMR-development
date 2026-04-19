# Useful Commands

## Quick Start (Top 4)
### 1) Jetson: AMR hardware bringup (motors + lidar)
```bash
docker run -d --name amr_foxy --net=host --privileged --runtime nvidia \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -v ~/AMR-development/ros_ws:/workspaces/ros_ws \
  amr/ros2-foxy-jetson:arm64 \
  bash -lc "ros2 launch amr_description hardware.launch.py \
    use_sim_time:=false \
    agent_dev:=/dev/ttyACM0 \
    agent_baud:=460800 \
    start_lidar:=true \
    start_camera:=false"
```

### 2) Dev PC: Start single container (for SLAM/NAV2 + RViz + Teleop)
```bash
pkill -f rviz2 || true
xhost +local:root
docker rm amr_devpc

docker run -it --name amr_devpc --net=host \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-foxy-devpc:amd64 \
  bash
```

### 3) Dev PC: Launch/Run robot state publisher, SLAM toolbox, RVIZ, Teleop, AMCL, Nav2 (inside amr_devpc container)

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

#Robot State Publisher
source /opt/ros/foxy/setup.bash; \
    ros2 run robot_state_publisher robot_state_publisher \
      --ros-args -p use_sim_time:=false \
      -p robot_description:="$(xacro /workspaces/AMR-development/ros_ws/src/amr_description/urdf/amr.urdf.xacro use_sim:=false)"
```



### 4) Load saved map (Nav2 AMCL localization)
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

### 5) Nav2 navigation (Map + AMCL + Planner/Controller)
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

## Networking & SSH
### Add SSH config alias (dev PC)
```bash
mkdir -p ~/.ssh
cat <<'SSHCONF' >> ~/.ssh/config

Host jetson
  HostName 192.168.1.9
  User kartik
  IdentitiesOnly yes
  IdentityFile ~/.ssh/id_ed25519
SSHCONF
chmod 600 ~/.ssh/config
```

### SSH to Jetson
```bash
ssh kartik@192.168.1.9
# alias (after adding ~/.ssh/config entry)
ssh jetson
```

## Jetson Runtime (on Jetson)
### Launch AMR hardware bringup
```bash
docker run --rm -it --net=host --privileged --runtime nvidia \
  -e ROS_DOMAIN_ID=0 \
  -v ~/AMR-development/ros_ws:/workspaces/ros_ws \
  --name amr_foxy \
  amr/ros2-foxy-jetson:arm64 \
  ros2 launch amr_description hardware.launch.py \
    use_sim_time:=false \
    agent_dev:=/dev/ttyACM0 \
    agent_baud:=460800 \
    start_lidar:=true \
    start_camera:=false
```

### Exec into running Jetson container
```bash
docker exec -it amr_foxy /entrypoint.sh bash
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

### AMR micro-ROS topic monitors
Use these during bench bring-up with only `micro_ros_agent` running.

```bash
# Wheel state
ros2 topic echo /amr/wheel_state sensor_msgs/msg/JointState --qos-reliability best_effort

# Safety / faults
ros2 topic echo /amr/fault_mask std_msgs/msg/Int32 --qos-reliability best_effort
ros2 topic echo /amr/safety_state std_msgs/msg/UInt32 --qos-reliability best_effort

# Duty commands
ros2 topic echo /amr/duty_cmd_left std_msgs/msg/Float32 --qos-reliability best_effort
ros2 topic echo /amr/duty_cmd_right std_msgs/msg/Float32 --qos-reliability best_effort

# Current (mA)
ros2 topic echo /amr/current_left_ma std_msgs/msg/Int32 --qos-reliability best_effort
ros2 topic echo /amr/current_right_ma std_msgs/msg/Int32 --qos-reliability best_effort
```

### AMR fault clear / safe state
```bash
ros2 topic pub --once /amr/enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /amr/estop std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /amr/clear_fault std_msgs/msg/Empty "{}"
```

### TF checks (best effort)
```bash
timeout 2s ros2 topic echo /tf --qos-reliability best_effort --qos-durability volatile

timeout 2s ros2 topic echo /tf_static --qos-reliability reliable --qos-durability transient_local
```

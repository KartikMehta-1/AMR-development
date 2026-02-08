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

### 1b) Jetson: AMR bringup (foreground, live logs)
```bash
docker run --rm -it --name amr_foxy --net=host --privileged --runtime nvidia \
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

### 2) Dev PC: Start single container (for SLAM + RViz + Teleop)
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

source /opt/ros/foxy/setup.bash; \
    ros2 run robot_state_publisher robot_state_publisher \
      --ros-args -p use_sim_time:=false \
      -p robot_description:="$(xacro /workspaces/AMR-development/ros_ws/src/amr_description/urdf/amr.urdf.xacro use_sim:=false)"
```


### 3) Dev PC: SLAM Toolbox (inside amr_devpc container)
```bash
source /opt/ros/foxy/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/slam_toolbox_online_async.yaml
```

### 3b) Dev PC: One-shot launch (RSP + SLAM + RViz with delays)
```bash
source /opt/ros/foxy/setup.bash
source /workspaces/AMR-development/ros_ws/install/setup.bash
ros2 launch amr_description bringup_slam_rviz.launch.py \
  use_sim_time:=false \
  use_sim:=false \
  slam_delay:=2.0 \
  rviz_delay:=4.0
```

### 4) Dev PC: RViz + Teleop (exec into same container)
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

### Stop Jetson container
```bash
docker stop amr_foxy
```

## Dev PC Runtime
### Single dev PC container for SLAM + RViz (avoid DDS issues)
```bash
# start a single shell container
docker run -it --name amr_devpc --net=host \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-foxy-devpc:amd64 \
  bash

# in container terminal 1
source /opt/ros/foxy/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/slam_toolbox_online_async.yaml

# in another host terminal
docker exec -it amr_devpc bash
source /opt/ros/foxy/setup.bash
rviz2
```

### RViz (host display)
```bash
pkill -f rviz2 || true
xhost +local:root

docker run --rm -it --net=host \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-foxy-devpc:amd64 \
  rviz2 -d /workspaces/AMR-development/ros_ws/src/amr_description/config/amr.rviz
```

### Robot state publisher (static TFs on dev PC)
```bash
docker run -d --name amr_rsp --net=host \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-foxy-devpc:amd64 \
  bash -lc 'source /opt/ros/foxy/setup.bash; \
    ros2 run robot_state_publisher robot_state_publisher \
      --ros-args -p use_sim_time:=false \
      -p robot_description:="$(xacro /workspaces/AMR-development/ros_ws/src/amr_description/urdf/amr.urdf.xacro use_sim:=false)"'
```

### Robot state publisher (launch file, if workspace built)
```bash
ros2 launch amr_description rsp.launch.py use_sim_time:=false use_sim:=false
```

### slam_toolbox (dev PC)
```bash
docker run -d --name slam_toolbox --net=host \
  -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-foxy-devpc:amd64 \
  ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=/workspaces/AMR-development/ros_ws/src/amr_description/config/slam_toolbox_online_async.yaml
```

### slam_toolbox logs
```bash
docker logs -f slam_toolbox
```

### Stop dev PC SLAM containers
```bash
docker rm -f amr_rsp slam_toolbox
```

## Docker Compose (dev PC SLAM stack)
### Start/stop amr_rsp + slam_toolbox
```bash
DEVPC_IP="$(ip -4 addr show wlan0 | awk '/inet /{print $2}' | cut -d/ -f1 | head -n1)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"${DEVPC_IP}\"/></Interfaces></General></Domain></CycloneDDS>"

docker compose -f ~/AMR-development/docker-compose.slam.yml up -d

docker compose -f ~/AMR-development/docker-compose.slam.yml down
```

## Teleop & Motion Tests
### Teleop (slow)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped \
  -p speed:=0.03 -p turn:=0.3
```

### Teleop (default)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### Timed cmd_vel (5s)
```bash
timeout 5s ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
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

### TF checks (best effort)
```bash
timeout 2s ros2 topic echo /tf --qos-reliability best_effort --qos-durability volatile

timeout 2s ros2 topic echo /tf_static --qos-reliability reliable --qos-durability transient_local
```

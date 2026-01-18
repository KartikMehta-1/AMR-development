# Useful Commands

## SSH to Jetson
```bash
ssh kartik@192.168.1.9
# alias (after adding ~/.ssh/config entry below)
ssh jetson
```

## Sync docker folder to Jetson
```bash
rsync -av --delete --exclude .git /home/kartik/AMR-development/docker/ kartik@192.168.1.9:~/AMR-development/docker/
```

## Build Jetson images from dev PC
```bash
ssh -t kartik@192.168.1.9 '
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

## Add SSH config alias (dev PC)
```bash
mkdir -p ~/.ssh
cat <<'EOF' >> ~/.ssh/config

Host jetson
  HostName 192.168.1.9
  User kartik
  IdentitiesOnly yes
  IdentityFile ~/.ssh/id_ed25519
EOF
chmod 600 ~/.ssh/config
```

## Run Jetson container
```bash
ssh -t kartik@192.168.1.9 '
docker run -it --rm \
  --name ros2_jetson \
  --net=host \
  --privileged \
  -v /dev:/dev \
  amr/ros2-foxy-jetson:arm64
'
```

## Exec into Jetson container
```bash
ssh -t kartik@192.168.1.9 'docker exec -it ros2_jetson /entrypoint.sh bash'
```

## Start micro-ROS agent in Jetson container
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 460800
```

## Start lidar in Jetson container
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

## Enable drive and clear faults
```bash
ros2 topic pub --once /amr/clear_fault std_msgs/msg/Empty "{}"
ros2 topic pub --once /amr/enable std_msgs/msg/Bool "{data: true}"
```

## Dev PC container for teleop
```bash
docker run -it --rm --name ros2_dev --net=host --privileged -v /dev:/dev amr/ros2-foxy-devpc:amd64
apt-get update && apt-get install -y ros-foxy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Stop Jetson container
```bash
ssh -t kartik@192.168.1.9 'docker stop ros2_jetson'
```

## Dev PC container for RViz
```bash
xhost +local:
docker run -it --rm --name ros2_dev --net=host \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  amr/ros2-foxy-devpc:amd64
rviz2 -d /opt/ros/rviz/amr_lidar.rviz
```

## Quick checks
```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list
ros2 node list
ls -l /dev/ttyACM* /dev/ttyUSB*
```

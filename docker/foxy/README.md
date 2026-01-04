# ROS 2 Foxy Docker Images

This folder defines a shared driver image and two runtime images:

1) `docker/foxy/Dockerfile` builds the **driver base image** with:
   - RealSense (librealsense + realsense-ros)
   - YDLidar G4 (ydlidar_ros2_driver)
   - micro-ROS agent
2) `docker/foxy/Dockerfile.devpc` builds the **dev PC image** with RViz2 + Gazebo.
3) `docker/foxy/Dockerfile.jetson` builds the **Jetson image** (headless).

## Build the driver base image

Dev PC (amd64):
```bash
docker buildx build -f docker/foxy/Dockerfile \
  --platform linux/amd64 \
  --build-arg BASE_IMAGE=ros:foxy-ros-base \
  -t amr/ros2-foxy-drivers:amd64 \
  --load .
```

Jetson Nano (arm64): replace the base image tag with your JetPack/L4T version.
```bash
docker buildx build -f docker/foxy/Dockerfile \
  --platform linux/arm64 \
  --build-arg BASE_IMAGE=dustynv/ros:foxy-ros-base-l4t-r32.7.1 \
  -t amr/ros2-foxy-drivers:arm64 \
  --load .
```

## Build the dev PC image (RViz2 + Gazebo)

```bash
docker buildx build -f docker/foxy/Dockerfile.devpc \
  --build-arg DRIVER_IMAGE=amr/ros2-foxy-drivers:amd64 \
  -t amr/ros2-foxy-devpc:amd64 \
  --load .
```

## Build the Jetson image (headless)

```bash
docker buildx build -f docker/foxy/Dockerfile.jetson \
  --build-arg DRIVER_IMAGE=amr/ros2-foxy-drivers:arm64 \
  -t amr/ros2-foxy-jetson:arm64 \
  --load .
```

## Run (example)

Dev PC with RViz2/Gazebo:
```bash
xhost +local:
docker run -it --rm --net=host \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/bus/usb \
  --device=/dev/ttyUSB0 \
  -v /home/kartik/AMR-development/ros_ws:/workspaces/ros_ws \
  amr/ros2-foxy-devpc:amd64
```

RViz2 (preconfigured LaserScan with best_effort QoS):
```bash
rviz2 -d /opt/ros/rviz/amr_lidar.rviz
```

Jetson (headless; add `--runtime nvidia` if you need GPU/camera acceleration):
```bash
docker run -it --rm --net=host \
  --device=/dev/bus/usb \
  --device=/dev/ttyUSB0 \
  -v /home/kartik/AMR-development/ros_ws:/workspaces/ros_ws \
  amr/ros2-foxy-jetson:arm64
```

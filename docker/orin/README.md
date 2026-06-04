# Jetson Orin NX ROS 2 Humble Docker Image

This profile is the AMR Orin runtime path. It is separate from `docker/foxy`
so the validated NUC + Jetson Nano/Foxy fallback remains intact.

Target role:

```text
Jetson Orin NX
  - ROS 2 Humble on JetPack 6 / L4T R36.x
  - micro-ROS agent for STM32
  - YDLidar driver
  - RealSense depth camera driver
  - ros2_control and diff_drive_controller
  - Nav2, AMCL, map_server, mission, and safety runtime
```

Build on the Orin:

```bash
cd ~/AMR-development
docker build -f docker/orin/Dockerfile -t amr/ros2-humble-orin:arm64 .
```

The current default base is `nvcr.io/nvidia/l4t-jetpack:r36.4.0`
because the exact `r36.5.0` L4T base tags were not published when checked from
the Orin. The host remains Jetson Linux R36.5.

The image builds librealsense from source using the RSUSB backend. The default
SDK tag tracks the minimum required by the selected `realsense-ros` branch
(`v2.57.7` for the current `ros2-development` branch). The YDLidar ROS 2 driver
is patched during the image build for ROS 2 Humble parameter declaration
compatibility.

Software-only AMR workspace build:

```bash
docker run --rm --net=host --runtime nvidia \
  -v ~/AMR-development:/workspaces/AMR-development \
  -w /workspaces/AMR-development/ros_ws \
  amr/ros2-humble-orin:arm64 \
  bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros/driver_ws/install/setup.bash && colcon build --merge-install --symlink-install'
```

Planning-only SO-101 MoveIt2 demo on the Orin:

```bash
docker run -it --rm --name amr_orin_moveit --net=host --privileged --runtime nvidia \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_LOCALHOST_ONLY=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-humble-orin:arm64 \
  bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros/driver_ws/install/setup.bash && cd /workspaces/AMR-development/ros_ws && colcon build --merge-install --symlink-install --packages-select amr_description amr_so101_moveit_config && source install/setup.bash && ros2 launch amr_so101_moveit_config demo.launch.py'
```

This MoveIt2 path is planning-only. It starts `move_group`, RViz, robot state
publisher, and joint-state GUI sliders; it does not define an arm hardware
driver, trajectory controller, gripper controller, or servo execution path.

Hardware-facing shell:

```bash
docker run -it --rm --name amr_orin --net=host --privileged --runtime nvidia \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_LOCALHOST_ONLY=0 \
  -v ~/AMR-development:/workspaces/AMR-development \
  amr/ros2-humble-orin:arm64
```

First hardware checks should be read-only or passive:

```bash
lsusb
ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 460800
ros2 topic list
```

Do not run motion, Nav2 goals, STM enable, fault clear, direct `/cmd_vel`, or
direct wheel commands until the Orin stack has passed supervised bring-up gates.

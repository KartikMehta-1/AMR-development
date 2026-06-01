# Orin AMR Useful Commands

These commands are for the Jetson Orin NX migration path.

Assumptions:

- NUC checkout: `/home/ubuntu/agent/repos/AMR-development`
- Orin SSH alias: `orin`
- Orin IP: `192.168.1.20`
- Orin project copy: `~/AMR-development`
- Orin runtime image: `amr/ros2-humble-orin:arm64`
- Orin hardware container: `amr_orin_hw`

## Start Hardware Runtime

Start the supervised Orin hardware stack with LiDAR, STM, controllers, and RealSense:

```bash
ssh orin 'docker rm -f amr_orin_hw >/dev/null 2>&1 || true; docker run -d --name amr_orin_hw --user root --net=host --privileged --runtime nvidia -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 -v /dev:/dev -v ~/AMR-development:/workspaces/AMR-development -w /workspaces/AMR-development/ros_ws amr/ros2-humble-orin:arm64 bash -lc "ros2 launch amr_description orin_hardware.launch.py start_camera:=true"'
```

Start the same stack with CycloneDDS pinned to the NUC peer. Use this after UDP Orin-to-NUC is cleared:

```bash
ssh orin 'docker rm -f amr_orin_hw >/dev/null 2>&1 || true; docker run -d --name amr_orin_hw --user root --net=host --privileged --runtime nvidia -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e AMR_CYCLONEDDS_PEER=192.168.1.8 -v /dev:/dev -v ~/AMR-development:/workspaces/AMR-development -w /workspaces/AMR-development/ros_ws amr/ros2-humble-orin:arm64 bash -lc "ros2 launch amr_description orin_hardware.launch.py start_camera:=true"'
```

Watch launch logs:

```bash
ssh orin 'docker logs -f --tail 120 amr_orin_hw'
```

Stop the hardware runtime:

```bash
ssh orin 'docker rm -f amr_orin_hw'
```

## NUC Operator Dashboard

Open the NUC-side tmux dashboard:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/open_amr_orin_dashboard.sh
```

Default mode is `remote`: panes run from the NUC terminal over SSH into the Orin hardware container. This is the reliable mode while Orin FastDDS discovery is still being tuned.

Experimental pure NUC subscriber mode:

```bash
AMR_ORIN_DASHBOARD_MODE=nuc ./scripts/open_amr_orin_dashboard.sh
```

The NUC mode uses a local `ros:humble-ros-base` monitor container on host networking. It is the desired longer-term operator-console shape, but it requires DDS discovery from the NUC to the Orin publishers to be clean.

Dashboard panes:

- Left column: grouped topics with publisher/subscriber counts.
- Middle column: grouped nodes with publisher/subscriber counts.
- Right column, top: compact wheel command, measured speed/rpm, current, and duty.
- Right column, bottom: STM comm state, safety state, fault masks, scan rate, and wheel-state rate.

The dashboard also creates a second tmux window named `lists` with static full topic, node, and service lists for scrolling/searching.

Tmux basics:

```text
Ctrl-b arrow  switch pane
mouse click    select pane
mouse wheel    scroll selected pane
Ctrl-b [       enter copy-mode scrolling
q              leave copy-mode
Ctrl-b n/p     next/previous window
Ctrl-b d      detach
tmux attach -t amr_orin_dashboard
```

## Teleop

Open Orin teleop from the NUC:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/open_amr_orin_teleop.sh
```

Default topic:

```text
/diff_drive_controller/cmd_vel_unstamped
```

Keys:

```text
i       forward
,       reverse
j / l   rotate
k       stop
Ctrl-C  exit
```

Keep the robot suspended or otherwise supervised until Orin hardware acceptance is complete.

## NUC RViz

Build the NUC RViz image:

```bash
cd /home/ubuntu/agent/repos/AMR-development
docker build -f docker/nuc-humble-rviz/Dockerfile -t amr/ros2-humble-rviz-nuc:amd64 .
```

Check basic UDP reachability. DDS needs Orin-to-NUC UDP to pass:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/check_amr_orin_udp.sh
```

Expected result:

```text
Testing UDP Orin -> NUC ... PASS ...
Testing UDP NUC -> Orin ... PASS ...
```

On 2026-06-01, both directions passed after clearing inbound UDP from the Orin on the NUC. If this regresses, reapply/check the NUC firewall rule:

```bash
sudo ufw allow in from 192.168.1.20 to any proto udp
sudo ufw reload
```

If UFW is inactive and the UDP check still fails, check NUC firewall/nftables rules or Wi-Fi client isolation. Plain UDP must work before RViz can receive `/scan`, TF, odometry, or camera topics from the Orin.

Open RViz on the NUC:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/open_amr_orin_rviz.sh
```

The launcher uses CycloneDDS by default, pins the NUC interface, and peers to `192.168.1.20`. It opens RViz with:

```text
/workspaces/AMR-development/ros_ws/src/amr_description/config/orin_rviz.rviz
```

RViz camera baseline:

- `RealSense Color` is enabled on `/camera/camera/color/image_raw`.
- `RealSense Depth` is present but disabled by default. Leave it unchecked unless explicitly debugging depth.
- Depth is currently diagnostic-only because the D455 depth path showed flashing/noisy behavior plus USB/control-transfer and Right MIPI warnings.

## Health Checks

Check the Orin hardware container:

```bash
ssh orin 'docker ps --filter name=amr_orin_hw --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"'
```

List devices on the Orin:

```bash
ssh orin 'lsusb; echo ---; ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB* /dev/video* 2>/dev/null || true'
```

Check controller state:

```bash
ssh orin 'docker exec amr_orin_hw bash -lc "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=$(ls /tmp/fastdds_*.xml | head -1); export ROS_DOMAIN_ID=0; source /opt/ros/humble/setup.bash; source /opt/ros/driver_ws/install/setup.bash; source /workspaces/AMR-development/ros_ws/install/setup.bash; ros2 control list_controllers --controller-manager /controller_manager"'
```

Check command topic subscription:

```bash
ssh orin 'docker exec amr_orin_hw bash -lc "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=$(ls /tmp/fastdds_*.xml | head -1); export ROS_DOMAIN_ID=0; source /opt/ros/humble/setup.bash; source /opt/ros/driver_ws/install/setup.bash; source /workspaces/AMR-development/ros_ws/install/setup.bash; ros2 daemon stop >/dev/null 2>&1 || true; ros2 daemon start >/dev/null 2>&1 || true; ros2 topic info -v /diff_drive_controller/cmd_vel_unstamped"'
```

Check topic rates:

```bash
ssh orin 'docker exec amr_orin_hw bash -lc "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=$(ls /tmp/fastdds_*.xml | head -1); export ROS_DOMAIN_ID=0; source /opt/ros/humble/setup.bash; source /opt/ros/driver_ws/install/setup.bash; source /workspaces/AMR-development/ros_ws/install/setup.bash; timeout 5 ros2 topic hz /amr_stm/wheel_state; timeout 5 ros2 topic hz /scan; timeout 5 ros2 topic hz /camera/camera/color/image_raw"'
```

## Build And Sync

Sync the NUC checkout to Orin:

```bash
rsync -av --delete --exclude build --exclude install --exclude log \
  /home/ubuntu/agent/repos/AMR-development/ \
  orin:~/AMR-development/
```

Build the AMR workspace inside the Orin image:

```bash
ssh orin 'docker run --rm --user root --net=host --privileged --runtime nvidia -v /dev:/dev -v ~/AMR-development:/workspaces/AMR-development -w /workspaces/AMR-development/ros_ws amr/ros2-humble-orin:arm64 bash -lc "source /opt/ros/humble/setup.bash; source /opt/ros/driver_ws/install/setup.bash; colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"'
```

Rebuild the Orin Docker image:

```bash
ssh orin 'cd ~/AMR-development && docker build -f docker/orin/Dockerfile -t amr/ros2-humble-orin:arm64 .'
```

## Latency Probe

The quick suspended probe on 2026-06-01 measured `/diff_drive_controller/cmd_vel_unstamped` to `/amr_stm/wheel_cmd_left/right` response mostly within roughly 20-75 ms. This reflects ROS/controller command generation, not human keypress timestamp or physical wheel acceleration.

For deeper latency work, compare:

- Terminal keypress to teleop publish.
- Teleop publish to `diff_drive_controller` subscriber.
- Controller update to `/amr_stm/wheel_cmd_*`.
- STM command receive to measured wheel velocity change.

The current controller update rate is 50 Hz, so one control cycle is about 20 ms.

## Current Pause

The next meaningful Orin step is mechanical/electrical:

- Print/install the Orin enclosure.
- Power the Orin from the AMR.
- Recheck cable routing and USB stability with the final mounting.
- Resume supervised floor hardware acceptance only after the Orin is mounted and powered on the robot.

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

RViz/camera viewing baseline:

- Keep raw camera topics local to the Orin for autonomy/training capture unless explicitly needed on the NUC.
- Use preview topics for Wi-Fi viewing:
  - `/camera/preview/color/image_raw` at 320x240, upright, about 5-6 Hz.
  - `/camera/preview/depth/image_rect_raw` at 320x240, upright, about 5-6 Hz.
  - `/so101/preview/wrist_camera/image_raw` at 320x240, about 8-10 Hz.
- Raw local topics remain available on the Orin:
  - `/camera/camera/color/image_raw`
  - `/camera/camera/depth/image_rect_raw`
  - `/so101/wrist_camera/image_raw`
- Depth is still diagnostic-only until the D455 depth path is validated under sustained operation.

Open preview viewers from the NUC:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/open_amr_camera_preview.sh
```

This direct OpenCV viewer subscribes to raw preview topics and avoids
`rqt_image_view` accidentally selecting compressed transports. If debugging
`rqt_image_view` specifically, the equivalent topic set is:

```bash
docker run --rm -it --name amr_camera_preview --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e AMR_CYCLONEDDS_PEER=192.168.1.20 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  amr/ros2-humble-rviz-nuc:amd64 \
  bash -lc 'ros2 run rqt_image_view rqt_image_view /camera/preview/depth/image_rect_raw & ros2 run rqt_image_view rqt_image_view /so101/preview/wrist_camera/image_raw; wait'
```

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

## SO-101 MoveIt Bridge

Build the SO-101 packages inside the Orin image:

```bash
ssh orin 'docker run --rm --user root --net=host --privileged --runtime nvidia -v /dev:/dev -v ~/AMR-development:/workspaces/AMR-development -w /workspaces/AMR-development/ros_ws amr/ros2-humble-orin:arm64 bash -lc "source /opt/ros/humble/setup.bash; source /opt/ros/driver_ws/install/setup.bash; colcon build --merge-install --symlink-install --packages-select amr_description amr_so101_driver amr_so101_moveit_config"'
```

For combined AMR base + SO-101 operation, start the AMR base joint states on a
base-only topic so the SO-101 merger can own global `/joint_states`:

```bash
ros2 launch amr_description orin_hardware.launch.py joint_states_topic:=/amr/joint_states
```

Start the MoveIt demo with the SO-101 bridge in fake mode:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=true
```

Start the real wrist-roll-only bridge after supervised confirmation:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=false \
  so101_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00 \
  driver_allowed_joints:=so101_wrist_roll
```

Start the real all-six bridge only for explicit supervised calibration:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=false \
  so101_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00 \
  driver_allowed_joints:=so101_shoulder_pan,so101_shoulder_lift,so101_elbow_flex,so101_wrist_flex,so101_wrist_roll,so101_gripper
```

Expected SO-101 interfaces:

```text
/so101_arm_controller/follow_joint_trajectory
/so101/joint_states
/amr/joint_states
/joint_states
```

Wrist-roll-only keyboard teleop, using the already-running MoveIt bridge:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/open_so101_wrist_roll_teleop.sh
```

Keys: `a`/left arrow rolls negative, `d`/right arrow rolls positive, `s` holds
current wrist roll, `[` and `]` adjust the step, and `q` exits. This helper only
sends `so101_wrist_roll` goals to `/so101_arm_controller/follow_joint_trajectory`.

Guarded SO-101 joint teleop:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/open_so101_joint_teleop.sh
```

Keys: `1` shoulder pan, `2` shoulder lift, `3` elbow flex, `4` wrist flex,
`5` wrist roll, `6` gripper, `a`/left arrow nudges negative, `d`/right arrow
nudges positive, `[` and `]` adjust the step, `p` prints current joint states,
and `q` exits. The default teleop sends one small trajectory point per keypress
for responsiveness; the bridge still enforces `max_step_rad` and start-state
tolerance.

SO-101 named poses:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/open_so101_named_pose.sh --list
./scripts/open_so101_named_pose.sh home
./scripts/open_so101_named_pose.sh neutral
./scripts/open_so101_named_pose.sh carry
./scripts/open_so101_named_pose.sh look_from_height
./scripts/open_so101_named_pose.sh ready_to_pick_up
```

`home` is the folded/stowed SO-101 pose. `neutral` is the older all-zero
calibration posture and should not be treated as the normal resting pose.

`all_servo_free` is not a joint pose; it releases servo torque through the
SO-101 bridge:

```bash
cd /home/ubuntu/agent/repos/AMR-development
./scripts/free_so101_servos.sh
```

Joint-limit calibration is still pending. The recorded `home`/`carry`,
`look_from_height`, and `ready_to_pick_up` poses came from physical encoder
readings, but some values are outside the starter URDF/MoveIt limits. Calibrate
by releasing servos, moving one joint at a time to safe physical endpoints,
recording encoder values, and then updating URDF, MoveIt, named-pose, and teleop
limits with safety margins.

Check wrist webcam enumeration before launching `usb_cam`:

```bash
ssh orin 'lsusb; ls -l /dev/video* /dev/v4l/by-id /dev/v4l/by-path 2>/dev/null || true; v4l2-ctl --list-devices 2>/dev/null || true'
```

## Current Checkpoint

The next meaningful Orin steps are supervised integration and calibration:

- Keep the mounted Orin powered with stable USB routing, cooling, and repeatable container restart behavior.
- Resume supervised floor hardware acceptance for the AMR base before promoting Orin over the NUC+Nano/Foxy fallback.
- Calibrate SO-101 physical joint limits before trusting named poses for autonomous manipulation.
- Measure SO-101 mount transform and wrist-camera extrinsics before grasp planning.

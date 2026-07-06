# Jetson Orin NX Device Profile

Owner: Kartik Mehta  
Device role: AMR upgraded on-robot compute, perception runtime, LeRobot/SO-101 runtime, future ROS 2 container host  
Last updated: 2026-06-01  
SSH aliases from laptop/NUC: `orin`, `jetson-orin`  
Static DHCP IP: `192.168.1.20`

This document is the local source of truth for the Jetson Orin NX currently being brought up for the AMR project. It separates the official module capability from the actual installed software and drivers observed on the device.

## Official Module Class

The module in use is treated as a Jetson Orin NX 16GB-class device.

Official NVIDIA Orin NX series specs relevant to this project:

- AI performance: up to 157 TOPS for Jetson Orin NX series.
- Power envelope: configurable between 10 W and 40 W.
- GPU: NVIDIA Ampere architecture GPU, Jetson Orin NX 16GB column lists 1024 CUDA cores and 32 Tensor Cores.
- CPU: 8-core Arm Cortex-A78AE v8.2 64-bit CPU, 2 MB L2 + 4 MB L3.
- Memory: 16 GB 128-bit LPDDR5, 102.4 GB/s.
- DLA/PVA: 1x NVDLA v2 and 1x PVA v2 on Orin NX.
- Camera interface: up to 4 cameras through 8 MIPI CSI-2 lanes, depending on carrier and virtual-channel configuration.
- PCIe: Orin NX module supports PCIe Gen4 lane configurations; actual carrier routing determines usable slots.
- USB/network/display availability depends on the carrier board.

Sources:

- NVIDIA Jetson Orin product/specification page: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/
- NVIDIA Jetson Orin NX 16GB availability/performance note: https://developer.nvidia.com/blog/boost-edge-ai-performance-with-the-new-nvidia-jetson-orin-nx-16gb/

## Observed Device Identity

Captured over SSH from `orin`.

```text
Hostname: kartik-Orin
Hardware vendor: NVIDIA
Hardware model: NVIDIA Jetson Orin NX Engineering Reference Developer Kit
Architecture: arm64 / aarch64
OS: Ubuntu 22.04.5 LTS (Jammy)
Kernel: Linux 5.15.185-tegra
L4T: R36 release, revision 5.0
Kernel variant: oot
```

`/etc/nv_tegra_release`:

```text
# R36 (release), REVISION: 5.0, GCID: 43688277, BOARD: generic, EABI: aarch64, DATE: Fri Jan 16 03:50:45 UTC 2026
# KERNEL_VARIANT: oot
TARGET_USERSPACE_LIB_DIR=nvidia
TARGET_USERSPACE_LIB_DIR_PATH=usr/lib/aarch64-linux-gnu/nvidia
```

## Installed Jetson Software State

The board was initially flashed with Jetson Linux only from SDK Manager. The full JetPack meta package was then installed from the NVIDIA Jetson R36.5 apt repository on 2026-05-08.

JetPack / Jetson Linux interpretation:

```text
SDK Manager selection: JetPack 6.2.2 Linux for Jetson Orin NX 16GB
Installed base OS: Jetson Linux / L4T R36.5.0
nvidia-jetpack meta package: installed
nvidia-jetpack installed version: 6.2.2+b24
```

Installed native JetPack components now include CUDA, cuDNN, TensorRT, VPI, NVIDIA OpenCV, multimedia APIs, and Nsight tools. Docker remains installed separately and has NVIDIA runtime support configured.

Observed package state:

```text
nvidia-jetpack:
  Installed: 6.2.2+b24
  Candidate: 6.2.2+b24

NVIDIA apt sources:
  https://repo.download.nvidia.com/jetson/common r36.5 main
  https://repo.download.nvidia.com/jetson/t234 r36.5 main
  https://repo.download.nvidia.com/jetson/ffmpeg r36.5 main
```

Key installed versions:

```text
CUDA Toolkit: 12.6
nvcc: 12.6.68 at /usr/local/cuda/bin/nvcc
cuDNN: libcudnn9-cuda-12 9.3.0.75-1
TensorRT: 10.3.0.30-1+cuda12.5
VPI: 3.2.4
NVIDIA OpenCV: 4.8.0-1-g6371ee1
```

Python import checks with system Python 3.10:

```text
cv2: 4.8.0
tensorrt: 10.3.0
vpi: 3.2.4
```

CUDA shell startup was added to `/home/kartik/.bashrc`:

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}
```

Fresh interactive shells should resolve `nvcc` directly. The absolute path also works:

```bash
/usr/local/cuda/bin/nvcc --version
```

## Power And Thermal Baseline

Current power mode:

```text
NV Power Mode: MAXN
```

Idle `tegrastats` sample:

```text
RAM 1693/15643MB
SWAP 0/7821MB
CPU: 8 cores present, light idle load around 729 MHz
GR3D_FREQ 0%
Temperatures: roughly 52-56 C at capture time
VDD_IN: about 6.9 W in the captured idle sample
```

Use this command during robotics or model tests:

```bash
tegrastats
```

## Storage

Root filesystem is on NVMe.

```text
Disk model: WD Blue SN5100 500GB
Root device: /dev/nvme0n1p1
Root size: 456 GB usable
Current usage after full JetPack install: 26 GB used, 408 GB free
```

Relevant `lsblk` summary:

```text
nvme0n1      WD Blue SN5100 500GB 465.8G disk
nvme0n1p1                         464.3G part ext4 /
nvme0n1p10                           64M part vfat /boot/efi
```

## Memory

Observed:

```text
RAM: 15 GiB visible to Linux
Swap: 7.6 GiB zram, unused at capture
```

`free -h` at capture:

```text
Mem: 15Gi total, 1.5Gi used, 11Gi free, 13Gi available
Swap: 7.6Gi total, 0B used
```

## Network

Static DHCP reservation:

```text
Orin NX Wi-Fi IP: 192.168.1.20
Laptop/NUC SSH aliases: orin, jetson-orin
```

NUC connectivity checkpoint on 2026-06-01:

```text
NUC host: ubuntu@ubuntu-EQ
NUC Wi-Fi IP: 192.168.1.8
Orin hostname: kartik-Orin
Orin IP: 192.168.1.20
Orin MAC seen from NUC: 14:75:5b:15:25:3e
SSH aliases added on NUC: orin, jetson-orin
SSH user: kartik
NUC key: /home/ubuntu/.ssh/id_ed25519
```

NUC SSH config entry:

```sshconfig
Host orin jetson-orin
  HostName 192.168.1.20
  User kartik
  IdentityFile ~/.ssh/id_ed25519
  IdentitiesOnly yes
```

Verification from the NUC:

```bash
ssh orin 'hostnamectl --static; hostname -I; cat /etc/nv_tegra_release | head -1'
```

## AMR Docker Runtime

Initial Orin runtime image built successfully on 2026-06-01.

```text
Image: amr/ros2-humble-orin:arm64
Image ID at latest build checkpoint: ada285aeee01
Approx size: 18.4 GB
Base image: nvcr.io/nvidia/l4t-jetpack:r36.4.0
Host L4T: R36.5.0
ROS distro: Humble
librealsense: 2.57.7, built from source with RSUSB backend
Driver overlay: /opt/ros/driver_ws/install
Included drivers: micro-ROS Agent, YDLidar ROS 2 driver, RealSense ROS driver
AMR workspace: /workspaces/AMR-development/ros_ws/install
```

Build command from the Orin:

```bash
cd ~/AMR-development
docker build -f docker/orin/Dockerfile -t amr/ros2-humble-orin:arm64 .
```

Smoke check used from the NUC:

```bash
ssh orin 'docker run --rm --net=host --runtime nvidia amr/ros2-humble-orin:arm64 bash -lc "echo ROS_DISTRO=\$ROS_DISTRO; ros2 pkg prefix ydlidar_ros2_driver; ros2 pkg prefix realsense2_camera; ros2 pkg prefix micro_ros_agent; rs-enumerate-devices --version"'
```

Expected checkpoint output:

```text
ROS_DISTRO=humble
/opt/ros/driver_ws/install
/opt/ros/driver_ws/install
/opt/ros/driver_ws/install
rs-enumerate-devices  version: 2.57.7.0
```

Notes:

- The default image base remains `l4t-jetpack:r36.4.0` because exact `r36.5.0` NGC image tags were not available when checked from the Orin.
- The YDLidar ROS 2 driver is patched during Docker build for ROS 2 Humble `declare_parameter` API compatibility and Humble launch-file argument names.
- RealSense D455 container bring-up works when the test container runs as root with `--privileged -v /dev:/dev`. Non-root container access currently sees the USB device but `librealsense` fails to open the USB interface; fix this with explicit USB device permissions/udev/container group handling before the production launch profile.
- The Orin image is not yet the primary AMR runtime. It must still pass AMR workspace build, lidar/micro-ROS device checks, supervised hardware bring-up, and acceptance tests.

AMR workspace build checkpoint on 2026-06-01:

```text
Workspace path on Orin: ~/AMR-development/ros_ws
Container image: amr/ros2-humble-orin:arm64, image ID ada285aeee01
Build command: colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
Result: 9 packages finished
Packages: amr_clients, amr_description, amr_hardware, amr_missions, amr_missions_msgs, amr_perception, amr_safety, amr_voice, my_pkg
Smoke checks: package prefixes resolve from install space, Xacro renders, hardware launch arguments render
```

Migration notes:

- `amr_hardware` was ported from the older Foxy-style `ros2_control` `SystemInterface` API to Humble lifecycle callbacks: `on_init`, `on_configure`, `on_activate`, `on_deactivate`, and `read/write(time, period)`.
- `amr_description/launch/hardware.launch.py` was updated to use the Humble controller-manager executable `spawner` instead of the Foxy-era `spawner.py`.
- Gazebo ROS dependencies in `amr_description` are sim-only for this Orin hardware bring-up. `ros-humble-gazebo-ros` and `ros-humble-gazebo-ros2-control` were not available from the current arm64 Humble apt repositories during rosdep install.
- `rosdep` reported unresolved `ament_python` keys for Python packages, but the workspace built because the Humble image already includes the needed `ament_python` build tooling.
- Added Orin-specific launch/config files: `amr_description/config/ydlidar_orin.yaml` and `amr_description/launch/orin_hardware.launch.py`. The wrapper uses stable by-id paths for the YDLidar CP2102 and STM ST-LINK serial devices.

Combined static bring-up checkpoint on 2026-06-01:

```text
Launch: ros2 launch amr_description orin_hardware.launch.py start_camera:=true
Robot state: physically suspended, supervised, no navigation, no manual wheel commands
Container image: amr/ros2-humble-orin:arm64, image ID ada285aeee01
Result after Humble spawner patch: launch remained up
Controllers: joint_state_broadcaster active, diff_drive_controller active
Hardware interfaces: left/right velocity command interfaces available and claimed; left/right position and velocity state interfaces available
LiDAR: /scan approximately 9.68 Hz
RealSense: depth and color image topics approximately 29-30 Hz
STM: required reset after agent start; /amr_stm/wheel_state approximately 9.79 Hz after reset
STM link status: /amr_stm/comm_status = stm_link_ok
Joint states: /joint_states present with left_wheel_joint and right_wheel_joint at zero position/velocity during suspended static test
```

NUC/RViz/Nav2 checkpoint on 2026-06-01:

```text
DDS: CycloneDDS peer path between NUC 192.168.1.8 and Orin 192.168.1.20
UDP: Orin -> NUC and NUC -> Orin checks passed after allowing inbound UDP from the Orin on the NUC firewall
NUC RViz image: amr/ros2-humble-rviz-nuc:amd64
RViz config: amr_description/config/orin_rviz.rviz
Nav2: Humble lifecycle nodes active after parameter/plugin updates
RViz goal path: /rviz and /rviz_navigation_dialog_action_client present as /navigate_to_pose action clients
Suspended Nav2 smoke: tiny CLI goal accepted and succeeded, and /amr_stm/wheel_cmd_left/right changed then returned to zero
Camera view: RealSense color visible in RViz; depth display kept disabled by default
```

Supervised suspended command-path checkpoint on 2026-06-01:

```text
Precondition: robot physically suspended and supervised
Fix required: stop/start the ROS 2 daemon in the Orin container after launch so the CLI sees the live graph
Command topic: /diff_drive_controller/cmd_vel_unstamped
Controller subscription: present, geometry_msgs/msg/Twist, use_stamped_vel=false
Test command: linear.x=0.04 m/s for 5 samples, then zero
Observed result: /amr_stm/wheel_cmd_left and /amr_stm/wheel_cmd_right rose to approximately 0.65 rad/s, then returned to zero
Helper: scripts/open_amr_orin_teleop.sh opens teleop through ssh -> docker exec with the runtime DDS environment detected from the hardware container
```

Observed issues and follow-ups:

- Old STM firmware may not reannounce XRCE entities if the Agent is restarted after the STM is already running. For now, start the Agent/launch first, then press STM reset if `/amr_stm/wheel_state` does not publish.
- The hardware plugin suppresses outgoing wheel commands while `/amr_stm/wheel_state` is stale or missing. During the successful combined check, the controllers were active only after supervised static setup, and no explicit motion command was sent.
- Orin teleop, dashboard, and RViz must use the same DDS family as the hardware container. Current helper scripts detect the runtime RMW where possible and use CycloneDDS for the NUC RViz path. If graph discovery looks stale, restart the ROS 2 daemon inside the container with the Orin ROS environment, then verify `ros2 topic info -v /diff_drive_controller/cmd_vel_unstamped` shows the `diff_drive_controller` subscription.
- YDLidar still emits repeated `Real points 931 > fixed points 930` warnings. This does not block initial bring-up, but it should be tuned before reliability clearance.
- The upstream YDLidar launch still emits a static `base_link -> laser_frame`; AMR TF should rely on URDF/`robot_state_publisher` for `base_link -> lidar_link`.
- RealSense depth remains diagnostic-only for now. Color is usable in RViz, but depth showed flashing/noisy behavior plus USB/control-transfer and Right MIPI warnings. Keep depth unchecked in RViz until the camera/cable/USB path is rechecked.
- Suspended wheel tests are useful only for command-path validation. They are not useful for localization quality because wheel odom changes while the room-relative LiDAR scan does not.
- Floor-motion testing is paused until one more revised Orin enclosure is printed/installed because the current design was insufficient, and the Orin can be powered from the AMR.

Active NetworkManager devices at capture:

```text
wlP1p1s0          wifi      connected               Airtel_keta_9683
docker0           bridge    connected externally    docker0
enP8p1s0          ethernet  unavailable
can0              can       unmanaged
usb0/usb1         ethernet  unmanaged
```

Wi-Fi hardware:

```text
PCI: Intel Corporation Wireless 8265 / 8275 (rev 78)
USB: Intel Bluetooth wireless interface 8087:0a2b
Driver modules loaded:
  iwlwifi
  iwlmvm
  mac80211
  cfg80211
```

Connection check:

```bash
ssh orin
hostname -I
nmcli device
```

## Docker And NVIDIA Container Runtime

Docker has been installed and verified on the Orin.

```text
Docker version: 29.1.3, build 29.1.3-0ubuntu3~22.04.2
docker service: active and enabled
User kartik is in docker group
hello-world container test: passed
```

Installed container packages:

```text
containerd                         2.2.1-0ubuntu1~22.04.1
docker.io                          29.1.3-0ubuntu3~22.04.2
libnvidia-container-tools          1.16.2-1
libnvidia-container1               1.16.2-1
nvidia-container-toolkit           1.16.2-1
nvidia-container-toolkit-base      1.16.2-1
runc                               1.3.4-0ubuntu1~22.04.1
```

Docker daemon runtime config:

```json
{
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    }
}
```

Default Docker runtime is still `runc`; use `--runtime nvidia` when GPU/NVIDIA runtime access is required:

```bash
docker run --rm --runtime nvidia <image> <command>
```

Basic checks:

```bash
docker ps
docker info | grep -i runtime
docker run --rm hello-world
```

## USB And Peripheral Baseline

USB devices observed at capture:

```text
Realtek 4-Port USB 3.0 Hub
Realtek 4-Port USB 2.0 Hub
Intel Bluetooth wireless interface
Logitech Unifying Receiver x2
```

AMR device checkpoint from the NUC on 2026-06-01:

```text
Intel RealSense D455: lsusb ID 8086:0b5c, detected on USB 3.2
D455 serial: 236322300171
D455 firmware: 5.16.0.1
librealsense recommended firmware: 5.17.0.10
CP210x UART bridge: lsusb ID 10c4:ea60, attached as /dev/ttyUSB0
CP210x stable path: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
D455 video nodes: /dev/video0 through /dev/video5
```

Container RealSense smoke test:

```bash
docker run --rm --net=host --runtime nvidia --privileged -v /dev:/dev --user root \
  amr/ros2-humble-orin:arm64 \
  bash -lc 'ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 enable_gyro:=false enable_accel:=false'
```

Observed ROS topics and rates:

```text
/camera/camera/depth/image_rect_raw: approximately 30 Hz
/camera/camera/color/image_raw: approximately 30 Hz
/camera/camera/depth/camera_info
/camera/camera/color/camera_info
/tf_static
```

Initial D455 reliability checkpoint on 2026-06-01:

```text
Test duration: 120 s topic-rate soak inside Docker
Depth rate: 29.77 Hz average, 3519-sample final window
Color rate: 29.56 Hz average, 3495-sample final window
USB topology: D455 on SuperSpeed path at 5000M through the USB3 hub
Thermals during camera-only test: roughly mid-50s C, no GPU load
Status: initial streaming pass with warnings
```

The RealSense ROS node came up and streamed depth/color near 30 Hz, but logs included repeated `control_transfer` warnings. Earlier short tests also produced one `Depth stream start failure` hardware notification. Treat this as functional for initial bring-up, not full reliability clearance. Follow up with non-root USB permissions, firmware/cable/hub/USB-load review, a 30-60 minute camera soak, lidar/micro-ROS concurrent load, and supervised robot acceptance before promoting Orin as the primary AMR runtime.

YDLidar G4 checkpoint on 2026-06-01:

```text
Device path: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
Baud: 230400
Detected model: G4
Serial: 2022021800075185
Firmware: 3.2
Hardware: 3
SDK: 1.2.20
ROS topic: /scan
Frame: lidar_link
Observed scan rate: approximately 9.67 Hz
Status: operational for initial bring-up, with warnings
```

The YDLidar driver connected and published `/scan` from inside the Orin Docker image. The upstream launch file originally failed on Humble due old `LifecycleNode` argument names; image `ada285aeee01` includes the Dockerfile patch and `ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=...` now starts successfully. The launch smoke test published `/scan` at approximately 9.68 Hz.

The upstream YDLidar launch still publishes a static transform from `base_link` to `laser_frame`, while the AMR scan frame is `lidar_link`. The AMR launch should rely on `robot_state_publisher`/URDF for the correct `base_link` to `lidar_link` transform instead of treating the driver launch's static TF as authoritative.

Driver logs showed initial checksum errors and repeated fixed-resolution warnings such as `Real points 931 > fixed points 930`. Treat this as usable for bring-up, but tune or soak-test before calling the lidar reliability-cleared. Candidate follow-ups: verify `fixed_resolution` behavior, run a 10-30 minute `/scan` soak, inspect range quality in RViz, and test concurrently with RealSense.

STM32 micro-ROS checkpoint on 2026-06-01:

```text
USB device: STMicroelectronics ST-LINK/V2.1, lsusb ID 0483:374b
Serial device: /dev/ttyACM0
Stable path: /dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066CFF34314B4E3043064322-if02
Agent command: micro_ros_agent serial --dev <stable path> -b 460800
Observed topics: /amr_stm/wheel_state, /amr_stm/ros_diag, /amr_stm/fault_mask, /amr_stm/safety_state, current/duty topics, and command topics
Wheel state rate: approximately 9.45 Hz
Fault mask: 0
Status: old Nano-era STM firmware connects to the Orin Docker micro-ROS Agent
```

This was a read-only connectivity check. No wheel commands, enable commands, e-stop commands, clear-fault commands, or navigation launch were sent.

After rebuilding the Docker image, the micro-ROS Agent executable still starts cleanly. The old STM firmware did not reannounce topics during a later agent-only restart without resetting the STM, so for repeated checks start the agent first and press reset/replug the STM if topics do not appear.

## SO-101 Manipulator Bench Checkpoint

The SO-101 follower controller has since been detected on the Orin:

```text
USB device: QinHeng Electronics USB Single Serial, lsusb ID 1a86:55d3
Serial device: /dev/ttyACM0
Stable path: /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00
```

Expected and observed follower motor IDs:

```text
1 shoulder_pan   STS3215 model 777
2 shoulder_lift  STS3215 model 777
3 elbow_flex     STS3215 model 777
4 wrist_flex     STS3215 model 777
5 wrist_roll     STS3215 model 777
6 gripper        STS3215 model 777
```

Read-only voltage/temperature/torque checks passed, and low-level wrist-roll
motion was validated through the Feetech/LeRobot bus. Shoulder lift moved
upward during torque/goal testing, so shoulder and elbow MoveIt execution remain
blocked until startup behavior is understood. The current software bridge is
`amr_so101_driver`, with real hardware execution gated to `so101_wrist_roll` by
default.

The wrist webcam software path is present (`usb_cam` and
`so101_wrist_webcam.launch.py`), but no `/dev/video*` device was detected before
shutdown. Recheck `lsusb`, `/dev/v4l/by-id`, and `v4l2-ctl --list-devices` when
the camera is plugged in.

User groups include access-oriented groups needed for robotics work:

```text
kartik adm cdrom sudo audio dip video plugdev render i2c lpadmin gdm sambashare docker weston-launch gpio
```

If serial access fails, add `dialout`:

```bash
sudo usermod -aG dialout $USER
```

Then log out and back in.

## Current AMR Bring-Up Implications

The Orin is ready for:

- SSH via `ssh orin`.
- Docker-based workflows.
- NVIDIA runtime containers with `--runtime nvidia`.
- Native JetPack 6.2.2 CUDA/cuDNN/TensorRT/VPI/OpenCV development.
- Wi-Fi operation through Intel AC8265.
- SO-101 through the reliable USB-A hub path at
  `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00`.
- Orin/container runtime validation of `amr_so101_driver`,
  `amr_so101_moveit_config`, low-bandwidth D455 previews, and SO-101 wrist
  webcam preview.

Still pending for AMR migration:

- Keep Orin power, USB routing, cooling, and container restart behavior stable on
  the mounted robot.
- Run supervised floor hardware acceptance with Orin physically mounted and
  powered from the robot.
- Keep the existing Nano/dev PC workflow on Foxy Docker until the migration is explicit and validated.
- Do not use host ROS on the dev PC as proof of Orin or Nano runtime compatibility.
- Recheck STM32 micro-ROS, LiDAR, RealSense, wrist webcam, and SO-101 USB device
  IDs after power loss or cable routing changes.
- Add Orin-specific launch variables/scripts instead of reusing the Nano host defaults.
- Calibrate SO-101 physical joint limits before using named poses for autonomous
  manipulation; all-six execution remains supervised bring-up/calibration only.

## Commands To Refresh This Profile

Run from the laptop:

```bash
ssh orin 'hostnamectl; cat /etc/os-release; cat /etc/nv_tegra_release; uname -a'
ssh orin 'nvpmodel -q; free -h; df -h; lsblk -o NAME,MODEL,SIZE,TYPE,FSTYPE,MOUNTPOINTS'
ssh orin 'hostname -I; nmcli device; lspci; lsusb; lsmod | grep -E "iwlwifi|iwlmvm|cfg80211|mac80211"'
ssh orin 'docker --version; docker ps; docker info | grep -i runtime; cat /etc/docker/daemon.json'
ssh orin 'apt-cache policy nvidia-jetpack; /usr/local/cuda/bin/nvcc --version'
ssh orin 'python3 - <<PY
for mod in ["cv2", "tensorrt", "vpi"]:
    m = __import__(mod)
    print(mod, getattr(m, "__version__", "imported"))
PY'
ssh orin 'tegrastats'
```

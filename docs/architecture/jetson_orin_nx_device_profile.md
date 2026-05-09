# Jetson Orin NX Device Profile

Owner: Kartik Mehta  
Device role: AMR upgraded on-robot compute, perception runtime, LeRobot/SO-101 runtime, future ROS 2 container host  
Last updated: 2026-05-08 06:38 IST  
SSH aliases from laptop: `orin`, `jetson-orin`  
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
Laptop SSH aliases: orin, jetson-orin
```

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

No SO-101 serial controllers were connected at the time of capture:

```text
/dev/serial/by-id/: empty or not present
```

Expected SO-101 IDs from the laptop setup notes, if the same controllers are plugged into Orin:

```text
Follower controller:
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00

Leader controller:
/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6053518-if00
```

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
- Direct USB serial tests for SO-101/LeRobot after the controllers are plugged in.

Still pending for AMR migration:

- Build or select an Orin/JetPack 6 container image for ROS 2 / AMR runtime.
- Define the Orin runtime as a separate Docker profile, likely ROS 2 Humble in Docker on JetPack 6.
- Keep the existing Nano/dev PC workflow on Foxy Docker until the migration is explicit and validated.
- Do not use host ROS on the dev PC as proof of Orin or Nano runtime compatibility.
- Verify STM32 micro-ROS, LiDAR, RealSense, and SO-101 USB device IDs on Orin.
- Add Orin-specific launch variables/scripts instead of reusing the Nano host defaults.
- Install or containerize LeRobot for SO-101 teleoperation.

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

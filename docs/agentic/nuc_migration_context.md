# NUC Migration Context

Owner: Runtime Environment Agent  
Secondary: ROS Core / Hardware Interface Agent, Navigation / Mission / Safety Agent  
Status: Active migration context  
Last Updated: 2026-06-01

This note captures the current context for moving AMR development and possibly robot-hosted runtime work from the older laptop + Jetson Nano split toward the NUC now available at `/home/ubuntu/agent/repos/AMR-development`.

## Current NUC Host Snapshot

- Host OS: Ubuntu 24.04.4 LTS (`noble`).
- Architecture: `x86_64`.
- User: `ubuntu`.
- Local checkout: `/home/ubuntu/agent/repos/AMR-development`.
- Git remote: `git@github.com:KartikMehta-1/AMR-development.git`.
- NUC Wi-Fi IP on AMR network: `192.168.1.8`.
- Jetson Orin NX SSH aliases on the NUC: `orin`, `jetson-orin` -> `kartik@192.168.1.20`.
- Docker status on 2026-05-30: installed and verified from an interactive shell after `newgrp docker`.
  - Docker Engine: `29.1.3-0ubuntu3~24.04.2`.
  - Docker Compose: `2.40.3+ds1-0ubuntu1~24.04.1`.
  - Docker Buildx: `0.30.1-0ubuntu1~24.04.1`.
  - `docker run --rm hello-world` passed and pulled the amd64 image from Docker Hub.
- `sudo` status on 2026-05-30: `ubuntu` is in the `sudo` group, but sudo requires an interactive password. Existing agent shells may not inherit the new Docker group until a fresh login or `newgrp docker` session.

## Existing Validated Runtime Shape

The repo currently assumes a validated split workflow:

```text
laptop/dev PC
  - Git/editing
  - amr/ros2-foxy-devpc:amd64 container
  - RViz, Nav2, mission server, safety supervisor, MCP servers
  - SSH to Jetson host alias: jetson

Jetson Nano on AMR
  - amr/ros2-foxy-jetson:arm64 container named amr_foxy
  - hardware.launch.py
  - micro-ROS agent on /dev/ttyACM0 at 460800 baud
  - LiDAR/camera/device access
  - ros2_control hardware interface and STM link watchdog
```

The standard launcher scripts reflect this split. In particular, `scripts/open_amr_devpc_navigation.sh` and `scripts/open_amr_devpc_slam.sh` SSH to `JETSON_HOST` (default `jetson`) to start the hardware-side `amr_foxy` container, then start the local `amr_devpc` container for navigation, mapping, mission, safety, RViz, and operator panes.

## NUC Migration Hypothesis

There are two reasonable NUC roles. Keep them separate until validated:

1. NUC as replacement dev PC
   - Run the current `amr/ros2-foxy-devpc:amd64` workflow on the NUC.
   - Keep Jetson Nano on the robot as the hardware runtime.
   - This is the lowest-risk migration path and should be validated first.

2. NUC as replacement robot computer
   - Physically mount/use the NUC on the AMR in place of the Jetson Nano.
   - Run an amd64 hardware-facing Foxy runtime locally on the NUC.
   - This requires new launch/runtime work because the existing scripts assume the hardware stack is remote on `jetson` and use the arm64 `amr/ros2-foxy-jetson:arm64` image.

Do not silently treat the NUC as equivalent to the Jetson Nano. The NUC is amd64, not arm64/L4T, and any hardware-facing migration must explicitly validate USB, serial, camera, LiDAR, power, network, thermal behavior, and ROS graph readiness.

## Docker Host Setup

Normal system Docker is installed. The install path should remain apt/systemd Docker, not Snap Docker, because AMR runtime needs host networking, USB devices, and predictable container behavior.

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2 docker-buildx tmux x11-xserver-utils openssh-client
sudo systemctl enable --now docker
sudo usermod -aG docker ubuntu
```

After logging out/in or running `newgrp docker`, verify:

```bash
docker version
docker compose version
docker buildx version
docker run --rm hello-world
```

This verification passed in the operator's interactive shell on 2026-05-30. If an agent shell still reports `permission denied while trying to connect to the docker API at unix:///var/run/docker.sock`, start a fresh shell session after group membership refresh before running Docker checks.

## Repo Path Implications

Many older docs show `~/AMR-development` or `/home/kartik/AMR-development`. The NUC checkout currently lives at:

```text
/home/ubuntu/agent/repos/AMR-development
```

The launcher scripts compute the local repo path dynamically and mount it into containers at `/workspaces/AMR-development`, so the main scripts are path-portable. However, some docs and `docker-compose.slam.yml` still use `~/AMR-development` examples. For the NUC, prefer script-based flows or update compose/docs before using compose.

## Validation Order

Use this order before any live hardware launch from the NUC:

1. Source-only repo checks.
2. Docker installed and hello-world verified from the shell that will run AMR commands.
3. Build the amd64 driver and dev-PC images.
4. Run software-only `colcon build` in the dev-PC container with no device mounts.
5. Validate map files, launch script preflight, host tools, and SSH reachability to the Jetson if using split mode.
6. If using NUC-as-dev-PC mode, launch only after explicit supervised confirmation because the standard launch starts hardware on the Jetson.
7. If using NUC-as-robot-computer mode, first design/update a dedicated amd64 hardware runtime path; do not reuse the Jetson arm64 image or remote-SSH launcher unchanged.

## 2026-05-30 NUC-To-Jetson Checkpoint

- GitHub update `4047ed8` was pulled on the NUC and added tracked map artifacts under `ros_ws/maps`.
  - `my_new_map.yaml` / `my_new_map.pgm`: `259 x 160`, resolution `0.05`, origin `[-3.72, -1.4, 0]`.
  - `my_hall_save.yaml` / `my_hall_save.pgm`: `216 x 299`, resolution `0.05`, origin `[-7.16, -7.53, 0]`.
  - `my_hall_save.posegraph` and `my_hall_save.data` are also present for SLAM Toolbox serialization.
- NUC Docker images are built:
  - `amr/ros2-foxy-drivers:amd64`
  - `amr/ros2-foxy-devpc:amd64`
- NUC software-only `colcon build --merge-install` passed in `amr/ros2-foxy-devpc:amd64`.
- NUC software-only `colcon test --merge-install` passed for available tests.
- `scripts/amr_static_map_publisher.py` successfully parsed and briefly published `my_new_map.yaml` in a no-hardware container check.
- Jetson SSH aliases `jetson` and `jetson-nano` resolve to `192.168.1.9` with user `kartik`; Jetson hostname reports `kartik-desktop`.
- Jetson reduced workspace currently contains `amr_description`, `amr_hardware`, and `amr_safety`.
- NUC-to-Jetson rsync has synced Docker files and the reduced runtime package sources to `~/AMR-development`.
- Jetson build-only container check passed:
  - image: `amr/ros2-foxy-jetson:arm64`
  - mounted workspace: `~/AMR-development/ros_ws` -> `/workspaces/ros_ws`
  - command: source `/opt/ros/foxy/install/setup.bash` and `/opt/ros/driver_ws/install/setup.bash`, then `colcon build --merge-install --symlink-install`
  - packages built: `amr_description`, `amr_hardware`, `amr_safety`
- Passive Jetson device preflight:
  - no AMR runtime container was running at check time.
  - `/dev/ttyACM0` exists as `root:plugdev`.
  - user `kartik` is in `plugdev`.
  - ST-LINK and CP210x USB devices were visible via `lsusb`.

## 2026-05-30 Live Split Bringup Checkpoint

- NUC is validated as a dev-PC replacement for the current split runtime, with Jetson Nano still acting as the hardware-side runtime.
- UFW on the NUC allows DDS UDP from Jetson `192.168.1.9` and local NUC traffic from `192.168.1.8`; raw UDP and ROS 2 graph discovery worked after these rules.
- Jetson `amr_foxy` hardware runtime is running and publishing:
  - `/amr_stm/fault_mask`: `0`
  - `/amr_stm/comm_status`: `stm_link_ok`
  - `/amr_stm/wheel_state`: present
  - `/scan`: present
  - `/odom`: present
  - controllers: `joint_state_broadcaster` and `diff_drive_controller` active
- NUC `amr_devpc` runtime is running Nav2 using `ros_ws/maps/my_new_map.yaml`.
- Operator set the initial pose in RViz.
- Localization readiness passed:
  - `scripts/amr_wait_for_localization.py --timeout 10`
  - AMCL pose near `x=3.715`, `y=0.480`, `yaw=3.023`
  - `map -> odom` TF available
- Nav2 action/lifecycle readiness passed for dry-run checks:
  - `/navigate_to_pose`, `/compute_path_to_pose`, `/follow_path`, `/spin`, and `/wait` actions visible
  - `/controller_server`, `/planner_server`, and `/bt_navigator` lifecycle states active
- Mission/safety bringup passed in read-only/monitor mode:
  - `mission_server` running and idle
  - named places loaded: `home`, `hall`, `door`, `kitchen`
  - `safety_supervisor` running with `monitor_only=true`, `enforce=false`, `require_amcl=false`
  - safety status healthy, `intervention_active=false`, `intervention_count=0`
- Supervised live mission validation passed after explicit operator confirmation:
  - guarded mission-control readiness for `hall` passed with no blockers
  - `go_to hall` was sent through `mission_server`, not raw `/cmd_vel` or a direct Nav2 action
  - mission state transitioned through `navigating` to `succeeded`
  - final detail: `reached 'hall'`
  - safety stayed healthy in monitor-only mode
  - STM fault mask stayed `0`, comm stayed `stm_link_ok`, and odom speed returned to zero
- Supervised two-leg route validation also passed:
  - guarded readiness for `kitchen` and `home` passed with no blockers
  - `go_to kitchen` succeeded with final detail `reached 'kitchen'`
  - after a second readiness check, `go_to home` succeeded with final detail `reached 'home'`
  - final mission state stayed `succeeded` at `home`
  - safety stayed healthy in monitor-only mode
  - STM fault mask stayed `0`, comm stayed `stm_link_ok`, and odom speed returned to zero
- The standard navigation launcher was repaired for tmux 3.4 by replacing old `split-window -p <percent>` pane sizing with `split-window -l <percent>%`.
- One-command NUC launch validation passed:
  - command shape: `AMR_ATTACH_TMUX=false AMR_REMOTE_REPO=/home/kartik/AMR-development AMR_SAFETY_ENFORCE=false ./scripts/open_amr_devpc_navigation.sh my_new_map`
  - Jetson `amr_foxy` started, STM reset completed, and controllers were active
  - NUC `amr_devpc_nav` tmux session was created with navigation and monitor windows
  - `/map` was ready from `my_new_map.yaml`
  - known `home` initial pose was published for validation
  - AMCL localization became ready near `x=3.510`, `y=0.517`, `yaw=3.118`
  - `mission_server` and `safety_supervisor` came up
  - guarded dry-run readiness for `hall` passed with no blockers
- No teleop, direct `/cmd_vel`, fault clear, safety reset, STM re-enable, or safety bypass was sent during this checkpoint.

## 2026-06-01 NUC-To-Orin Connectivity Checkpoint

- The Jetson Orin NX is reachable from the NUC at `192.168.1.20`.
- Device identity:
  - hostname: `kartik-Orin`
  - Jetson Linux: R36 release, revision 5.0
  - architecture: arm64 / aarch64
- NUC SSH aliases were added in `/home/ubuntu/.ssh/config`:

```sshconfig
Host orin jetson-orin
  HostName 192.168.1.20
  User kartik
  IdentityFile ~/.ssh/id_ed25519
  IdentitiesOnly yes
```

- Verification passed from the NUC:

```bash
ssh orin 'hostnamectl --static; hostname -I'
ssh jetson-orin 'cat /etc/nv_tegra_release | head -1'
```

- The `.local` mDNS names `kartik-Orin.local` / `kartik-orin.local` may resolve intermittently from the NUC, so project scripts should prefer the stable SSH aliases or the static DHCP IP.

## Safety Boundary

Installing Docker, building images, reading code, and software-only builds are non-motion tasks.

Starting hardware-facing containers, running `hardware.launch.py`, starting Nav2 on the real robot, launching SLAM with teleop panes, clearing faults, resetting safety intervention, enabling STM, teleop, or missions all still require explicit supervised confirmation in the current interaction.

The NUC migration must preserve the existing motion path:

```text
mission/operator
-> Nav2 / mission_server
-> diff_drive_controller
-> amr_hardware
-> /amr_stm/wheel_cmd_left/right
-> STM firmware
-> Cytron MDD20A
```

It must not introduce direct `/cmd_vel`, direct wheel velocity, direct PWM, or safety-bypass paths.

## Open Design Questions

- Will the NUC remain off-robot as the dev-PC replacement, or move onto the AMR as the primary robot computer?
- If the NUC goes on the robot, which USB devices connect directly to it: STM serial, LiDAR, RealSense, ST-LINK, audio, IMU, or other peripherals?
- Should the Jetson Nano remain as a fallback hardware runtime during the transition?
- Should a new `amr/ros2-foxy-nuc:amd64` image/profile exist, or should the dev-PC image grow a hardware-facing mode?
- Does the NUC need X11/RViz locally, headless operation, or both?
- Which network interface should DDS bind to when the NUC is on the robot?

# AMR Safety Baseline

This baseline is the known-good operating record to capture before enabling or changing safety layers. The goal is to prove that the robot, STM firmware, Jetson link, odometry, lidar, localization, and navigation are stable without the ROS safety supervisor adding intervention.

## Current Baseline Scope

- STM firmware safety checks may remain enabled.
- ROS safety supervisor is disabled unless explicitly started.
- Navigation is launched with the normal map and mission stack.
- The baseline must pass while idle, during motion, and after motion.

## Record The Setup

Fill this in for each baseline run:

```text
date/time:
top-level branch:
top-level commit:
STM firmware branch/commit:
STM flashed from:
map:
battery level/voltage:
floor / obstacle conditions:
reset button / NRST wiring state:
STM 5V/GND/USB cable notes:
Jetson container/image notes:
ROS safety supervisor state:
operator notes:
```

Useful commands:

```bash
git rev-parse --abbrev-ref HEAD
git rev-parse --short HEAD
git submodule status --recursive
```

## Launch

From the laptop:

```bash
cd ~/AMR-development
./scripts/open_amr_devpc_navigation.sh my_new_map
```

Wait for localization to become valid:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && python3 ../scripts/amr_wait_for_localization.py --timeout 180 --print-period 2'
```

Confirm the ROS safety supervisor is not running during this baseline:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'source /workspaces/AMR-development/ros_ws/install/setup.bash && ros2 node list | grep amr_safety || true'
```

## Baseline Probe

The probe checks topic rates, stale gaps, STM fault state, communication fault state, STM reboot symptoms, and basic command/current ranges. It uses compatible QoS for the STM best-effort topics.

Idle probe:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && python3 ../scripts/amr_baseline_probe.py --duration 60'
```

Motion probe, in one terminal:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && python3 ../scripts/amr_baseline_probe.py --duration 240'
```

While the motion probe is running, execute one or more missions from another terminal:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && ros2 run amr_missions mission_cli go_to kitchen'
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && ros2 run amr_missions mission_cli go_to hall'
```

Post-motion probe:

```bash
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && python3 ../scripts/amr_baseline_probe.py --duration 60'
```

Optional log capture:

```bash
mkdir -p logs/safety_baseline
docker exec amr_devpc /entrypoint.sh bash -lc 'cd /workspaces/AMR-development/ros_ws && source install/setup.bash && python3 ../scripts/amr_baseline_probe.py --duration 60' | tee logs/safety_baseline/idle_$(date +%Y%m%d_%H%M%S).txt
```

## Expected Healthy Values

- `/amr_stm/wheel_state`: about 9-10 Hz, no stale gap above 0.5 s.
- `/amr_stm/fault_mask`: about 9-10 Hz, value `0`, no stale gap above 0.5 s.
- `/amr_stm/ros_diag`: about 2 Hz, UART write failures, write timeouts, and publisher failures remain `0`.
- `/amr_stm/comm_status`: about 2 Hz, value `stm_link_ok`.
- `/amr_stm/comm_fault_mask`: value `0`.
- `/odom`: about 50 Hz, no stale gap above 0.5 s.
- `/scan`: about 9-10 Hz, no stale gap above 0.5 s.
- `/amr_stm/safety_state`: may contain state bits, but should not accompany a nonzero fault mask during baseline operation.
- STM `boot_ms` must not drop during idle or motion. A drop means the STM rebooted.

Launch-time reset cause bits can reflect the launch script, reset button, or ST-LINK reset path. Do not treat launch-time `SOFT` or `PIN` reset cause as a failure by itself. Treat a runtime boot counter drop as a failure.

## Pass Criteria

- Idle probe reports `PASS`.
- Motion probe reports `PASS` while the AMR is teleoperated or executing missions.
- Post-motion probe reports `PASS`.
- Missions complete without fault, cancel, or stall.
- RViz pose remains physically plausible while moving and after stopping.
- No STM reboot is observed during motion.
- No nonzero STM fault mask or communication fault mask is observed.

## Failure Clues

- `boot_ms drops`: STM rebooted. Check reset cause, 5V supply, GND, NRST/reset wiring, ST-LINK/reset button wiring, and firmware hangs.
- `fault_mask != 0`: STM safety fault is active. Decode the mask before clearing it.
- `comm_fault_mask != 0` or `comm_status != stm_link_ok`: Jetson/STM communication is not healthy.
- Wheel state or fault mask gaps: STM publisher or micro-ROS agent path is unstable.
- Odom gaps: controller/hardware interface path is unstable.
- Scan gaps: lidar path is unstable.
- Large AMCL steps during motion with stable odom/scan: localization tuning or TF timing needs investigation.
- Large AMCL steps during motion with odom or wheel gaps: fix hardware/STM/control data first.

## Before Adding Each Safety Layer

For every new safety feature or threshold change:

1. Capture an idle baseline.
2. Capture a motion baseline.
3. Enable only one new safety layer.
4. Repeat the same idle and motion probes.
5. Record whether the new layer observed, warned, limited, or stopped the AMR.
6. Keep the layer disabled by default until its trigger condition and recovery path are repeatable.

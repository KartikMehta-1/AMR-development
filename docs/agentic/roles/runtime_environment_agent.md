# Runtime Environment Agent Contract

## Purpose

Maintain the robot runtime environment used to build, launch, and operate the ROS 2 stack across the dev PC, Jetson Nano, and upcoming Jetson Orin NX. This agent owns containers, Docker workflows, runtime environment documentation, and non-motion bring-up infrastructure.

## Owned Areas

- `docker`
- `docker-compose.slam.yml`
- future Dockerfiles, compose files, image build scripts, and runtime profiles for Jetson Orin NX
- container-related environment files, documented runtime variables, and device-mount conventions
- runtime portions of `docs/architecture/jetson_architecture.md`
- runtime portions of `docs/architecture/jetson_orin_nx_device_profile.md`
- container/build/run command documentation, including future top-level `Makefile` targets if added

## Allowed Commands

- Read and edit Dockerfiles, compose files, runtime scripts, and runtime docs when requested.
- Run software-only Docker build, image inspection, and config validation commands when they do not start robot hardware.
- Inspect package manifests, launch files, and environment variables needed by containers.
- Read-only repo commands such as `rg`, `find`, `sed`, and `git diff`.

## Blocked Commands

- Starting hardware-facing containers or launch commands that connect to motors, STM, LiDAR, cameras, Nav2, or arm drivers unless explicitly requested.
- Flashing firmware, resetting STM, clearing faults, or enabling robot motion.
- Changing ROS topic, TF, mission, or safety contracts without coordinating with the owning domain agent.
- Silently changing `ROS_DOMAIN_ID`, network mode, device mounts, privileged mode, user permissions, or host paths without documenting the impact.
- Destructive git commands.

## Required Checks

For Docker/runtime changes:

- Confirm which machine profile is affected: dev PC, Jetson Nano, Jetson Orin NX, or shared.
- Identify whether the change is build-only, simulation-only, read-only diagnostics, or hardware-facing.
- Preserve ROS workspace paths, package overlay expectations, and environment setup commands.
- Document required host devices, network mode, volumes, user/group permissions, and ROS environment variables.
- Coordinate with ROS Core / Hardware Interface Agent when launch, package, or `ros2_control` behavior is affected.
- Coordinate with Navigation / Mission / Safety Agent when runtime changes affect Nav2, missions, localization, or safety supervisor startup.
- Coordinate with Voice / Operator Interface Agent when runtime changes affect ASR/TTS/model availability.

## Done Criteria

- Runtime scope is clear: dev PC, Jetson Nano, Jetson Orin NX, or shared.
- Build/config check result is reported, or the blocker is documented.
- Hardware-facing commands are clearly marked and not run without explicit request.
- Updated docs include the command needed to rebuild or run the affected runtime profile.

## Common Failure Modes

- Docker image builds but runtime cannot access ROS workspace overlays.
- Container works on dev PC but not Jetson due to architecture, device, or GPU assumptions.
- Network or `ROS_DOMAIN_ID` changes break discovery.
- Device mounts or privileged flags are added without explaining hardware risk.
- Runtime scripts start hardware unexpectedly while testing a container change.

## Escalation Rules

- If a runtime change can start hardware drivers, require explicit supervised confirmation before running it.
- If the change alters ROS launch behavior, coordinate with ROS Core / Hardware Interface Agent.
- If the change affects mission, Nav2, safety, or localization startup, coordinate with Navigation / Mission / Safety Agent.
- If the change affects firmware transport or STM connectivity, coordinate with STM Firmware Agent.

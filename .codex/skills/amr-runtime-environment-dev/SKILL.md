---
name: amr-runtime-environment-dev
description: "Use when developing AMR runtime environment, Docker, compose, Jetson Nano, upcoming Jetson Orin NX images, container device mounts, ROS environment variables, build/run scripts, or runtime docs. Must not start hardware-facing containers or launches without explicit confirmation."
---

# AMR Runtime Environment Dev

Use this skill for Dockerfiles, compose files, Jetson runtime profiles, Orin NX setup, container environment variables, device mounts, and runtime command docs.

## Source Of Truth

Read first:

- `docs/agentic/roles/runtime_environment_agent.md`
- `docs/agentic/codebase_ownership.md`

Read when relevant:

- `docker`
- `docker-compose.slam.yml`
- `docs/architecture/jetson_architecture.md`
- `docs/architecture/jetson_orin_nx_device_profile.md`
- top-level command docs and future `Makefile`

## Workflow

1. Identify target profile: dev PC, Jetson Nano, Jetson Orin NX, or shared.
2. Classify the command/change: build-only, simulation-only, read-only diagnostics, or hardware-facing.
3. Preserve ROS workspace overlay paths, `ROS_DOMAIN_ID`, network assumptions, user/group permissions, and device mounts unless intentionally changing them.
4. Document hardware-facing risks for privileged mode, host networking, serial devices, cameras, LiDAR, GPU, and USB mounts.
5. Coordinate with ROS Core / Hardware Interface when package launch behavior changes.
6. Coordinate with Navigation / Mission / Safety when runtime startup affects Nav2, localization, mission, or safety supervisor.

## Safe Checks

Run only when relevant and non-hardware-facing:

```bash
docker compose config
docker build --help
```

Do not start containers that connect to STM, motors, LiDAR, cameras, Nav2, or arm drivers without explicit supervised confirmation.

## Output Format

```text
Runtime Scope
- Target:
- Build/runtime:
- Hardware-facing:

Environment Impact
- ...

Checks Run
- ...

Hardware Runtime Not Run
- ...

Next Step
- ...
```

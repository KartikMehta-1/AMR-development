# Runtime Environment Architecture

Owner: Runtime Environment Agent  
Secondary: ROS Core / Hardware Interface Agent  
Status: Active

This block owns how the AMR software is built and run across the dev PC, Jetson Nano, and upcoming Jetson Orin NX.

## Responsibility

- Keep AMR ROS execution containerized.
- Treat Foxy Docker as the authoritative current Nano/dev PC workflow.
- Treat host ROS on the dev PC as non-authoritative unless explicitly requested.
- Keep Orin NX migration as a separate Humble Docker profile, not a silent replacement for Nano/Foxy.
- Maintain Dockerfiles, runtime scripts, device-mount policy, and runtime environment docs.

## Runtime Block Diagram

```mermaid
flowchart TB
  DEVPC[Dev PC / Laptop\nDocker host, Git, editor, SSH]
  NANO[Jetson Nano\nFoxy hardware runtime]
  ORIN[Jetson Orin NX\nfuture Humble runtime]

  subgraph FoxyCurrent[Current Foxy Docker Profiles]
    FOXY_DEV[amr/ros2-foxy-devpc:amd64\nsoftware validation, RViz/Gazebo]
    FOXY_JET[amr/ros2-foxy-jetson:arm64\nheadless robot runtime]
    FOXY_DRV[amr/ros2-foxy-drivers\nshared driver base]
  end

  subgraph OrinFuture[Future Orin Profile]
    HUMBLE_ORIN[Humble + JetPack 6 container\nplanned]
  end

  DEVPC --> FOXY_DEV
  DEVPC -->|build/test current stack| FOXY_DEV
  NANO --> FOXY_JET
  FOXY_DRV --> FOXY_DEV
  FOXY_DRV --> FOXY_JET
  ORIN --> HUMBLE_ORIN
```

## Detailed Sources

- `docker/foxy/README.md`
- `docs/architecture/jetson_architecture.md`
- `docs/architecture/jetson_orin_nx_device_profile.md`
- `docs/agentic/roles/runtime_environment_agent.md`

## Validation

- Source-only harness checks may run in CI.
- Current AMR ROS build validation should run inside `amr/ros2-foxy-devpc:amd64`.
- Hardware-facing containers with device mounts, host networking, or privileged access require explicit supervised confirmation.

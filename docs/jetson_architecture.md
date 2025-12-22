# Jetson Nano Architecture (Compute + ROS 2 Runtime)

Owner: Kartik Mehta  
Status: Draft — containerized ROS 2 Humble runtime on L4T 32.7.x with micro-ROS agent.  
Last Updated: 2025-12-22

## Goals
- Run ROS 2 Humble on Jetson Nano despite Ubuntu 18.04 base by using L4T containers.
- Host micro-ROS agent and AMR ROS nodes; provide GPU-accelerated perception where needed.
- Keep runtime reproducible between dev PC and Jetson via Docker/Compose.

## Platform
- Hardware: Jetson Nano Dev Kit.
- OS: JetPack 4.x (Ubuntu 18.04, L4T r32.7.x).
- Container runtime: Docker + NVIDIA container runtime.

## Runtime Stack (Containerized)
- Base image: L4T Humble ROS 2 image (e.g., dustynv/ros:humble-ros-base-l4t-r32.7.1).
- micro-ROS agent runs in the same container or a separate agent container.
- Host networking enabled for DDS and XRCE-DDS discovery.
- Volumes for logs, rosbags, and configs mounted from the host.

## Hardware Interfaces
- LiDAR: USB via powered hub.
- Depth camera: RealSense D455 via USB 3.
- Proximity sensors: routed through STM32 (micro-ROS client publishes ranges).
- Motor control: STM32 over micro-ROS (XRCE-DDS) via serial transport to agent.

## ROS 2 Graph Responsibilities
- micro-ROS agent (XRCE-DDS) bridging MCU topics into the ROS 2 graph.
- odometry, safety_monitor, sensor_fusion, and navigation nodes.
- rosbag recording and diagnostics for field tests.

## Services and Startup
- systemd unit or docker-compose to auto-start the runtime stack on boot.
- Health checks: ros2 doctor, topic list, agent connectivity, GPU availability.

## Networking
- Static/reserved IP or mDNS for PC <-> Jetson connectivity.
- Time sync via NTP; configure ROS_DOMAIN_ID if multiple robots.

## Validation Checklist
- Docker container starts with --runtime nvidia and --network host.
- ros2 topic list works in container.
- micro-ROS agent accepts STM32 connection and relays /cmd_vel and wheel RPM topics.
- LiDAR scan and depth camera topics visible; RViz visualization on PC.

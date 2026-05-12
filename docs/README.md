# AMR Documentation Index

This folder is organized by documentation purpose.

## Project

- `project/AMR_project.md` - canonical roadmap and project tracker
- `project/AMR_firmware_tasks.tmp` - obsolete pointer retained for old references

## Architecture

- `architecture/README.md` - architecture index and agent-owned diagram map
- `architecture/00_system_hierarchy.md` - top-level AMR hierarchy from operator/agent input to actuators and sensor feedback
- `architecture/10_runtime_environment.md` - Docker, dev PC, Jetson Nano, Orin NX, Foxy/Humble split
- `architecture/20_ros_core_hardware_interface.md` - ROS core, `ros2_control`, hardware bridge, TF, STM topic bridge
- `architecture/30_navigation_mission_safety.md` - SLAM, AMCL, Nav2, mission, safety
- `architecture/40_stm_firmware.md` - STM32 firmware architecture entry point
- `architecture/50_voice_operator_interface.md` - voice/text operator interface architecture
- `architecture/60_manipulator_moveit.md` - SO-101 manipulator, MoveIt, guarded execution architecture
- `architecture/70_perception_calibration.md` - RGB-D perception, calibration, and proposal architecture
- `architecture/80_physical_hardware.md` - physical hardware architecture summary
- `architecture/ros_stack_diagrams.md` - ROS 2 graph, TF ownership, navigation stack
- `architecture/STM_architecture.md` - STM32 firmware architecture
- `architecture/jetson_architecture.md` - Jetson runtime architecture
- `architecture/jetson_orin_nx_device_profile.md` - Jetson Orin NX device profile

## Hardware

- `hardware/hardware_block_diagram.md` - hardware block diagram
- `hardware/Component_specifications.md` - component specifications
- `hardware/pin_map.yaml` - authoritative pin map
- `hardware/wiring_schematic.md` - wiring plan
- `datasheets/` - vendor datasheets

## Safety

- `safety/safety_baseline.md` - safety baseline procedure/reporting
- `safety/safety_fault_recovery.md` - fault decode and recovery procedure
- `safety/safety_supervisor_step*.md` - safety-supervisor implementation and validation steps

## Agentic Tooling

- `agentic/agentic_behavior_diagram.md` - diagram of agents, skills, MCPs, harnesses, ROS clients, and robot runtime boundaries
- `agentic/agentic_robotics_roadmap.md` - agentic robotics implementation roadmap
- `agentic/agent_tool_permissions.md` - agent permission classes and blocked actions
- `agentic/agent_interaction_examples.md` - example interactions
- `agentic/codebase_ownership.md` - repo area ownership by agent
- `agentic/roles/` - subagent role contracts

## Perception

- `perception/vla_manipulation_layers.md` - RGB-D, VLA, MCP, MoveIt, and guarded execution boundary model

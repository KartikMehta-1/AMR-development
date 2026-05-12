# Codebase Ownership

This document maps repo areas to the current agent structure. It is a routing aid for future work, not a permission override. The permission model in `docs/agentic/agent_tool_permissions.md` still controls what agents may run, especially around hardware and motion.

## Ownership Status

- `active`: current source of truth or active development area.
- `future`: planned area with an owner, but not fully implemented yet.
- `archived`: retained for reference; changes should be rare and explicitly scoped.
- `artifact`: generated, downloaded, binary, map, model, or exported asset.
- `unassigned`: intentionally not governed by a project agent yet.

## Primary Ownership Matrix

| Area | Primary Agent | Secondary Agent | Status | Notes |
| --- | --- | --- | --- | --- |
| `STM/STM_Firmware_AMR_v2` | STM Firmware Agent | ROS Core / Hardware Interface Agent | active | Active STM32 firmware source of truth. Coordinate topic, unit, fault-mask, and timing changes with ROS docs and clients. |
| `STM/STM_Firmware_` | STM Firmware Agent | Code Review Agent | archived | Legacy STM firmware reference. Do not treat as current behavior unless explicitly requested. |
| `STM/STM_MotorCheck` | STM Firmware Agent | Code Review Agent | archived | Legacy or bench firmware. Use only for reference or explicitly scoped bench work. |
| `STM/STM_SRM_CurrentBench` | STM Firmware Agent | Code Review Agent | archived | Legacy/current-sensor bench reference. Keep separate from active firmware behavior. |
| `STM_Firmware_AMR_v2` | STM Firmware Agent | Code Review Agent | archived | Root-level legacy copy or old workspace. Active firmware remains under `STM/STM_Firmware_AMR_v2`. |
| `ros_ws/src/amr_hardware` | ROS Core / Hardware Interface Agent | STM Firmware Agent | active | Hardware bridge, watchdog, and STM topic contracts. |
| `ros_ws/src/amr_clients` | ROS Core / Hardware Interface Agent | Navigation / Mission / Safety Agent | active | Shared ROS client helpers for mission, safety, localization, navigation, robot health, and STM diagnostics. |
| `ros_ws/src/amr_description` | ROS Core / Hardware Interface Agent | Navigation / Mission / Safety Agent | active | URDF, launch, controller config, Nav2/SLAM/AMCL runtime wiring. |
| `ros_ws/src/amr_missions` | Navigation / Mission / Safety Agent | Voice / Operator Interface Agent | active | Persistent mission runtime and named-place behavior. |
| `ros_ws/src/amr_missions_msgs` | ROS Core / Hardware Interface Agent | Navigation / Mission / Safety Agent | active | Shared message/service contracts. |
| `ros_ws/src/amr_safety` | Navigation / Mission / Safety Agent | STM Firmware Agent | active | Safety supervisor and robot-side safety state handling. |
| `ros_ws/src/amr_voice` | Voice / Operator Interface Agent | Navigation / Mission / Safety Agent | active | Voice/text intent layer over mission and safety. |
| `ros_ws/src/amr_perception` | Perception / Calibration Agent | Manipulator / MoveIt Agent | active | RGB-D perception contracts and proposal helpers. Outputs are not actuator commands. |
| `ros_ws/src/amr_semantic_nav` | Navigation / Mission / Safety Agent | Perception / Calibration Agent | active | Navigation-owned for now; coordinate with perception if semantic perception/VLM behavior becomes central. |
| `ros_ws/src/my_pkg` | Code Review Agent | Test Runner Agent | archived | Appears to be a stray/tutorial package. Do not expand unless it is reclassified. |
| `ros_ws/maps` | Navigation / Mission / Safety Agent | Runtime Environment Agent | artifact | Map artifacts used by localization/navigation. Track provenance when maps become release artifacts. |
| `ros_ws/build`, `ros_ws/install`, `ros_ws/log` | Test Runner Agent | Runtime Environment Agent | artifact | Generated ROS workspace output. Do not edit manually. |
| `scripts` | Navigation / Mission / Safety Agent | Test Runner Agent | active | Mission, safety, localization, monitor, and bring-up scripts. Hardware-facing scripts require explicit confirmation before running. |
| `docker` | Runtime Environment Agent | ROS Core / Hardware Interface Agent | active | Dockerfiles and runtime images for dev PC, Jetson Nano, and upcoming Jetson Orin NX. |
| `docker-compose.slam.yml` | Runtime Environment Agent | Navigation / Mission / Safety Agent | active | Runtime orchestration; do not start hardware-facing services without explicit request. |
| `mcp_servers/amr_state_inspection` | ROS Core / Hardware Interface Agent | Navigation / Mission / Safety Agent | active | Read-only robot health, safety, localization, mission, places, STM diagnostics, navigation, and last-known-place inspection. |
| `mcp_servers/amr_mission_control` | Navigation / Mission / Safety Agent | ROS Core / Hardware Interface Agent | active | Guarded named-place mission MCP. Motion-capable calls require readiness checks and explicit supervised confirmation. |
| `mcp_servers/amr_robot_launch` | Runtime Environment Agent | Navigation / Mission / Safety Agent | active | Guarded host-side launch MCP for standard AMR navigation runtime. Live launch requires explicit supervised confirmation. |
| `mcp_servers/amr_voice_interface` | Voice / Operator Interface Agent | Navigation / Mission / Safety Agent | active | Input-agnostic transcript-to-intent MCP. It recommends safe next tool calls but does not execute motion. |
| `mcp_servers/amr_conversation` | Voice / Operator Interface Agent | Code Review Agent | active | Stateless conversation turn planner returning responses and safe MCP tool plans. |
| `mcp_servers/amr_speaker` | Voice / Operator Interface Agent | Runtime Environment Agent | active | Spoken-feedback MCP publishing to `/amr_voice/say`; it does not decide robot actions. |
| `mcp_servers/amr_perception_inspection` | Perception / Calibration Agent | Manipulator / MoveIt Agent | active | Read-only RGB-D/camera/scene/object/grasp proposal MCP. It must not command motion or manipulation. |
| other `mcp_servers` | Runtime Environment Agent | ROS Core / Hardware Interface Agent | active | New MCP servers need explicit ownership, permission class, and smoke tests before use. |
| `docs/architecture/README.md` | ROS Core / Hardware Interface Agent | all domain agents | active | Architecture index and agent-owned diagram routing map. |
| `docs/architecture/00_system_hierarchy.md` | ROS Core / Hardware Interface Agent | all domain agents | active | Top-level communication hierarchy from high-level input to actuators and sensor feedback. Keep this stable and link block-level diagrams from it. |
| `docs/architecture/10_runtime_environment.md` | Runtime Environment Agent | ROS Core / Hardware Interface Agent | active | Runtime environment architecture entry point for Docker, dev PC, Nano, and Orin profiles. |
| `docs/architecture/20_ros_core_hardware_interface.md` | ROS Core / Hardware Interface Agent | STM Firmware Agent | active | ROS core and hardware interface architecture entry point. |
| `docs/architecture/30_navigation_mission_safety.md` | Navigation / Mission / Safety Agent | ROS Core / Hardware Interface Agent | active | Navigation, mission, localization, and safety architecture entry point. |
| `docs/architecture/40_stm_firmware.md` | STM Firmware Agent | ROS Core / Hardware Interface Agent | active | STM firmware architecture entry point. Detailed source remains `STM_architecture.md`. |
| `docs/architecture/50_voice_operator_interface.md` | Voice / Operator Interface Agent | Navigation / Mission / Safety Agent | active | Voice/text operator interface architecture entry point. |
| `docs/architecture/60_manipulator_moveit.md` | Manipulator / MoveIt Agent | ROS Core / Hardware Interface Agent | active | SO-101 manipulator, MoveIt, guarded execution, and VLA proposal handoff architecture entry point. |
| `docs/architecture/70_perception_calibration.md` | Perception / Calibration Agent | Manipulator / MoveIt Agent | active | RGB-D perception, calibration, read-only perception MCP, and proposal-output architecture entry point. |
| `docs/perception` | Perception / Calibration Agent | Manipulator / MoveIt Agent | active | Perception, VLA, dataset, and proposal/execution boundary notes. |
| `docs/architecture/80_physical_hardware.md` | STM Firmware Agent | Runtime Environment Agent | active | Physical hardware architecture summary that links to detailed hardware docs. |
| `docs/architecture/STM_architecture.md` | STM Firmware Agent | ROS Core / Hardware Interface Agent | active | Firmware architecture and micro-ROS contract docs. |
| `docs/architecture/ros_stack_diagrams.md` | ROS Core / Hardware Interface Agent | Navigation / Mission / Safety Agent | active | ROS graph, topic ownership, TF, and stack wiring. |
| `docs/architecture/jetson_architecture.md` | Runtime Environment Agent | ROS Core / Hardware Interface Agent | active | Jetson runtime architecture. |
| `docs/architecture/jetson_orin_nx_device_profile.md` | Runtime Environment Agent | ROS Core / Hardware Interface Agent | active | Orin NX runtime/device profile. |
| `docs/agentic/agentic_behavior_diagram.md` | Code Review Agent | Test Runner Agent | active | Agentic behavior diagram covering agents, skills, MCP servers, harnesses, shared ROS clients, and robot runtime boundaries. |
| `docs/hardware` | STM Firmware Agent | Runtime Environment Agent | active | Pin map is STM-owned; hardware block/spec docs may be cross-domain references. |
| `docs/safety` | Navigation / Mission / Safety Agent | STM Firmware Agent | active | Safety baseline, fault recovery, and safety-supervisor procedures. |
| `docs/project` | Code Review Agent | all domain agents | active | Project tracker and planning docs. Domain owners update their own sections; review agent checks consistency. |
| `docs/agentic` | Code Review Agent | Test Runner Agent | active | Agent contracts, permissions, skills roadmap, and this ownership map. |
| `.codex/skills` | Test Runner Agent | Code Review Agent | active | Repo-local skills. Domain-specific skills should cite the relevant role contract. |
| future `agent_harness` | Test Runner Agent | Code Review Agent | future | Agent/harness scenarios for software, simulation, and supervised hardware acceptance. |
| `models` | Voice / Operator Interface Agent | Runtime Environment Agent | artifact | ASR/TTS/model artifacts. Track model version, source, and runtime requirements. |
| `Workspace` | Code Review Agent | owning domain agent by topic | archived | Experimental tuning, sensor, and analysis workspace. Promote useful scripts/docs into governed locations before relying on them. |
| `CAD` | unassigned | Code Review Agent | unassigned | Intentionally not governed for now. Mechanical/CAD ownership can be added later. |
| `PCB` | unassigned | Code Review Agent | unassigned | Intentionally not governed for now. Electrical/PCB ownership can be added later. |
| `.vscode` | Runtime Environment Agent | Code Review Agent | active | Developer environment settings when they affect build/debug workflows. |
| top-level command/docs files | Runtime Environment Agent | Code Review Agent | active | Includes `useful commands.md`, `Folder_Structure.md`, future `Makefile`, and runtime command docs. |

## Routing Rules

- If a task changes active STM firmware behavior, route to STM Firmware Agent.
- If a task changes the ROS hardware bridge, URDF, controller config, package metadata, launch wiring, or shared ROS interfaces, route to ROS Core / Hardware Interface Agent.
- If a task changes Nav2, SLAM, AMCL, mission runtime, safety supervisor, named places, recovery behavior, or safety scripts, route to Navigation / Mission / Safety Agent.
- If a task changes Docker, image builds, runtime profiles, Jetson setup, container environment variables, device mounts, or upcoming Orin NX runtime work, route to Runtime Environment Agent.
- If a task changes voice, parser, ASR/TTS, wake-word, confirmation, or operator feedback behavior, route to Voice / Operator Interface Agent.
- If a task changes future arm URDF, MoveIt, arm drivers, gripper, trajectory, or manipulation planning, route to Manipulator / MoveIt Agent.
- If a task changes calibration, camera/depth processing, perception proposals, datasets, or image-processing code, route to Perception / Calibration Agent.
- If a task asks for validation, tests, builds, linting, or harness scenarios, route to Test Runner Agent.
- If a task asks for review, regression risk, or consistency across changed files, route to Code Review Agent.

## Cross-Agent Requirements

- Topic, service, action, frame, unit, and fault-mask changes require both the owning implementation agent and the affected consumer agent.
- Hardware-facing commands require explicit user confirmation even if the owning agent allows edits in that area.
- Archived firmware and workspace folders may be inspected, but should not be treated as source of truth without explicit reclassification.
- Generated artifacts should not be manually edited unless the task is specifically about artifact management.

## Runtime Validation Policy

- The authoritative AMR ROS environment is containerized.
- Current Nano/dev PC validation uses the Foxy Docker profile, especially `amr/ros2-foxy-devpc:amd64` for software-only checks.
- Host ROS installations on the dev PC are not authoritative for AMR validation. If host ROS is used for a quick inspection, report it explicitly and do not treat it as proof that the current robot workflow passes.
- Orin NX work should use a separate runtime profile, likely Humble in Docker on JetPack 6, and should not silently replace the current Foxy/Nano workflow.

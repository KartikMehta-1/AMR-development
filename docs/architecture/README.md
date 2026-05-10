# Architecture Index

This folder is organized around agent-owned system responsibilities. The older detailed documents remain in place where they are already used by skills, harness checks, and project references; the numbered files below are the clean entry points for each block.

## Agent-Owned Architecture Set

| File | Primary Owner | Secondary Owner | Purpose |
| --- | --- | --- | --- |
| `00_system_hierarchy.md` | ROS Core / Hardware Interface Agent | all domain agents | Top-level communication path from high-level input to actuators and sensor feedback. |
| `10_runtime_environment.md` | Runtime Environment Agent | ROS Core / Hardware Interface Agent | Docker, dev PC, Jetson Nano, Orin NX, Foxy/Humble split, runtime profiles. |
| `20_ros_core_hardware_interface.md` | ROS Core / Hardware Interface Agent | STM Firmware Agent | ROS graph boundary, `ros2_control`, `amr_hardware`, TF, STM topic bridge. |
| `30_navigation_mission_safety.md` | Navigation / Mission / Safety Agent | ROS Core / Hardware Interface Agent | SLAM, AMCL, Nav2, mission server, safety supervisor, named places. |
| `40_stm_firmware.md` | STM Firmware Agent | ROS Core / Hardware Interface Agent | STM32 firmware, motor control, micro-ROS, E-stop, fault masks. |
| `50_voice_operator_interface.md` | Voice / Operator Interface Agent | Navigation / Mission / Safety Agent | Voice/text operator path, parser, confirmation, mission client use. |
| `60_manipulator_moveit.md` | Manipulator / MoveIt Agent | ROS Core / Hardware Interface Agent | Future SO-101 arm, URDF, MoveIt, gripper, guarded execution. |
| `70_perception_calibration.md` | Perception / Calibration Agent | Manipulator / MoveIt Agent | Future camera/depth perception, calibration, object/grasp proposals. |
| `80_physical_hardware.md` | STM Firmware Agent | Runtime Environment Agent | Power, wiring, sensors, drive hardware, physical integration. |

## Current Detailed Diagram Homes

| Existing diagram source | Agent-owned home | Notes |
| --- | --- | --- |
| `ros_stack_diagrams.md` full ROS graph | `20_ros_core_hardware_interface.md` | Keep detailed topic tables here until split. |
| `ros_stack_diagrams.md` base motion and odometry | `20_ros_core_hardware_interface.md` | Core bridge from Nav2/teleop to `amr_hardware` and STM. |
| `ros_stack_diagrams.md` TF ownership | `20_ros_core_hardware_interface.md` | Shared with Navigation / Mission / Safety Agent. |
| `ros_stack_diagrams.md` mapping mode | `30_navigation_mission_safety.md` | SLAM and map creation workflow. |
| `ros_stack_diagrams.md` localization/navigation mode | `30_navigation_mission_safety.md` | AMCL, map server, Nav2 planning/control. |
| `ros_stack_diagrams.md` mission layer over Nav2 | `30_navigation_mission_safety.md` | Mission server and named-place behavior. |
| `STM_architecture.md` diagrams | `40_stm_firmware.md` | STM firmware remains the detailed source of truth. |
| `hardware_block_diagram.md` | `80_physical_hardware.md` | Physical block summary remains under `docs/hardware`. |
| `wiring_schematic.md` diagrams | `80_physical_hardware.md` | Wiring details remain under `docs/hardware`. |

## Rule

Architecture docs describe responsibility boundaries, communication paths, and contracts. Hardware docs describe component-level wiring and physical implementation details. When a change crosses a boundary, update both the agent-owned architecture page and the detailed source document.

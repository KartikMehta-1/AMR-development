# ROS Core And Hardware Interface Architecture

Owner: ROS Core / Hardware Interface Agent  
Secondary: STM Firmware Agent  
Status: Active

This block owns the ROS boundary between application-level commands and the physical base interface.

## Responsibility

- `ros2_control`, `controller_manager`, `diff_drive_controller`, and `amr_hardware`.
- URDF/Xacro, controller config, launch wiring, TF ownership, and package metadata.
- Shared ROS interfaces and clients that other tools use.
- The bridge from ROS wheel commands to STM micro-ROS topics.

## Core Interface Diagram

```mermaid
flowchart LR
  subgraph Inputs[Command Sources]
    TELEOP[Teleop]
    NAV2[Nav2 controller]
    MISSION[Mission server via Nav2]
  end

  subgraph ROSControl[ROS Control]
    DDC[diff_drive_controller]
    CM[controller_manager]
    HW[amr_hardware]
    WATCHDOG[amr_link_watchdog]
  end

  subgraph MicroROS[micro-ROS Bridge]
    AGENT[micro_ros_agent]
    STM[STM micro-ROS client]
  end

  subgraph Feedback[Feedback]
    WHEELSTATE[/amr_stm/wheel_state]
    ODOM[/diff_drive_controller/odom]
    TF[/tf and /tf_static]
    DIAG[/amr_stm diagnostics]
  end

  TELEOP --> DDC
  NAV2 --> DDC
  MISSION --> NAV2
  CM --> DDC
  DDC --> HW
  HW -->|/amr_stm/wheel_cmd_left/right| AGENT
  AGENT --> STM
  STM --> WHEELSTATE
  WHEELSTATE --> AGENT
  AGENT --> HW
  HW --> DDC
  DDC --> ODOM
  DDC --> TF
  WATCHDOG --> DIAG
```

## Detailed Sources

- `docs/architecture/ros_stack_diagrams.md`
- `ros_ws/src/amr_description`
- `ros_ws/src/amr_hardware`
- `ros_ws/src/amr_clients`
- `ros_ws/src/amr_missions_msgs`
- `docs/architecture/STM_architecture.md`

## Current Detailed Diagrams Owned Here

- Full real-hardware ROS graph.
- Base motion and odometry path.
- TF ownership.
- Nav2 remapping and topic ownership tables.

## Validation

- Foxy Docker `colcon build` for affected ROS packages.
- No hardware launch, motor driver, Nav2 mission, or motion command without explicit supervised confirmation.

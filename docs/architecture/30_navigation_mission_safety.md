# Navigation, Mission, And Safety Architecture

Owner: Navigation / Mission / Safety Agent  
Secondary: ROS Core / Hardware Interface Agent  
Status: Active

This block owns navigation behavior above the core hardware interface.

## Responsibility

- SLAM, AMCL, Nav2 planning/control/recoveries, maps, and named places.
- Mission server and mission CLI behavior.
- Safety supervisor state, motion denial logic, and recovery procedures.
- Readiness checks for localization, safety, mission, and navigation.

## Navigation And Mission Diagram

```mermaid
flowchart TB
  OP[Operator / Agent Request]
  CLIENTS[Shared ROS Clients]
  MISSION[mission_server]
  SAFETY[safety_supervisor]
  PLACES[places.yaml]
  NAV2[Nav2 BT Navigator]
  PLANNER[planner_server]
  CONTROLLER[controller_server]
  COSTMAPS[global + local costmaps]
  LOCALIZE[SLAM / AMCL / map_server]
  BASE[diff_drive_controller path]

  OP --> CLIENTS
  CLIENTS --> MISSION
  CLIENTS --> SAFETY
  PLACES --> MISSION
  SAFETY -->|allow / block| MISSION
  MISSION -->|NavigateToPose goals| NAV2
  LOCALIZE --> NAV2
  NAV2 --> PLANNER
  NAV2 --> CONTROLLER
  COSTMAPS --> PLANNER
  COSTMAPS --> CONTROLLER
  CONTROLLER --> BASE
```

## Detailed Sources

- `docs/architecture/ros_stack_diagrams.md`
- `docs/safety/safety_baseline.md`
- `docs/safety/safety_fault_recovery.md`
- `ros_ws/src/amr_missions`
- `ros_ws/src/amr_safety`
- `ros_ws/src/amr_description/config/nav2_params_amr.yaml`
- `ros_ws/src/amr_missions/config/places.yaml`

## Current Detailed Diagrams Owned Here

- Mapping mode with SLAM Toolbox.
- Localization and navigation mode.
- Mission layer over Nav2.

## Validation

- Software-only parser/config/unit tests when available.
- Foxy Docker build/test for affected packages.
- Simulation only when explicitly requested.
- Physical navigation or recovery only with supervised hardware confirmation.

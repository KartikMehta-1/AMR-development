# AMR System Hierarchy Diagram

This is the top-level communication hierarchy for the AMR project. It shows how high-level operator or agent input flows down to robot motion, and how sensor/diagnostic feedback flows back up.

The LLM/agent layer does not directly command motors. It can call bounded tools, such as MCP tools or CLI workflows, which then pass through ROS clients, mission/safety logic, Nav2, `ros2_control`, the STM32 firmware, and finally the motor driver.

## Detailed Diagram Inventory

Detailed architecture and hardware diagram sources that existed before the agent-owned entry-point split:

| Document | Scope | Mermaid diagrams |
| --- | --- | --- |
| `docs/architecture/ros_stack_diagrams.md` | ROS graph, TF, mapping, localization, navigation, mission layer, topic ownership | 6 |
| `docs/architecture/STM_architecture.md` | STM32 firmware, micro-ROS, control loop, fault states | 4 |
| `docs/hardware/hardware_block_diagram.md` | Power, compute, sensors, drive hardware | 1 |
| `docs/hardware/wiring_schematic.md` | Wiring and signal-level hardware details | 2 |

Total detailed Mermaid diagrams in those source documents: 13.

## High-Level Input-To-Actuator And Sensor Feedback Path

```mermaid
flowchart TB
  subgraph HumanAgentLayer[High-Level Input And Agent Layer]
    OPERATOR[Operator]
    VOICE[Voice / Text Interface]
    LLM[LLM / Codex / Agent]
    SKILLS[AMR Skills]
    MCP[MCP Tools]
    MCPVOICE[voice / conversation / speaker MCPs]
    MCPSTATE[state inspection MCP]
    MCPMISSION[mission control MCP]
    MCPLAUNCH[robot launch MCP]
    HARNESS[Harness / Self-Tests]
  end

  subgraph AppLayer[Application And Robot API Layer]
    CLI[Mission CLI / Scripts]
    ROSCLIENTS[Shared ROS Clients]
    MISSION[Mission Server]
    SAFETY[Safety Supervisor]
  end

  subgraph PlanningLayer[Planning, Localization, And Runtime Layer]
    NAV2[Nav2 BT / Planner / Controller / Recoveries]
    MOVEIT[MoveIt2 / SO-101 Planning]
    LOCALIZE[SLAM / AMCL / Map Server]
    COSTMAPS[Global + Local Costmaps]
    RSP[Robot State Publisher / TF]
    RUNTIME[Docker Runtime\nFoxy fallback / Humble Orin]
  end

  subgraph ROSControlLayer[ROS Control And Hardware Interface Layer]
    DDC[diff_drive_controller]
    CM[controller_manager]
    HW[amr_hardware]
    ARMDRV[amr_so101_driver]
    JSMERGE[joint_state_merger]
    WATCHDOG[amr_link_watchdog]
    AGENT[micro_ros_agent]
  end

  subgraph FirmwareLayer[STM32 Firmware Layer]
    UROS[STM micro-ROS Client]
    FWSTATE[Enable / E-stop / Fault State]
    CONTROL[100 Hz Wheel Speed PI]
    SENSE[Encoder + Current Sense]
  end

  subgraph PhysicalLayer[Physical Robot Layer]
    DRIVER[Cytron MDD20A Motor Driver]
    MOTORS[Left / Right Drive Motors]
    ARM[SO-101 Arm]
    WRISTCAM[SO-101 Wrist Webcam]
    ENCODERS[Wheel Encoders]
    ESTOP[Physical E-stop PB10]
    LIDAR[YDLidar G4]
    CAMERA[RealSense D455]
    IMU[BNO080 IMU]
  end

  OPERATOR --> VOICE
  OPERATOR --> CLI
  OPERATOR --> LLM
  LLM --> SKILLS
  SKILLS --> MCP
  MCP --> MCPVOICE
  MCP --> MCPSTATE
  MCP --> MCPMISSION
  MCP --> MCPLAUNCH
  HARNESS -.validates.-> SKILLS
  HARNESS -.validates.-> MCP

  VOICE --> MCPVOICE
  MCPVOICE --> ROSCLIENTS
  MCPVOICE --> MCPSTATE
  MCPVOICE --> MCPMISSION
  MCPVOICE -->|spoken response| VOICE
  CLI --> ROSCLIENTS
  MCPSTATE --> ROSCLIENTS
  MCPMISSION --> ROSCLIENTS
  MCPLAUNCH -.guarded standard runtime launch.-> RUNTIME
  ROSCLIENTS --> MISSION
  ROSCLIENTS --> SAFETY

  MISSION -->|named goals / patrol sequencing| NAV2
  SAFETY -->|health / intervention state| ROSCLIENTS
  NAV2 --> COSTMAPS
  LOCALIZE --> NAV2
  RSP --> NAV2
  RUNTIME --> MISSION
  RUNTIME --> SAFETY
  RUNTIME --> NAV2
  RUNTIME --> MOVEIT

  NAV2 -->|cmd_vel remapped to controller input| DDC
  MOVEIT -->|/so101_arm_controller/follow_joint_trajectory| ARMDRV
  DDC -->|left/right wheel command interfaces| HW
  CM --> DDC
  HW -->|/amr_stm/wheel_cmd_left/right| AGENT
  WATCHDOG -->|/amr_stm/comm_status + fault mask| SAFETY

  AGENT --> UROS
  UROS --> FWSTATE
  FWSTATE --> CONTROL
  CONTROL --> DRIVER
  DRIVER --> MOTORS
  ARMDRV -->|Feetech serial bus| ARM

  MOTORS --> ENCODERS
  ARM -->|/so101/joint_states| JSMERGE
  HW -->|/amr/joint_states in combined mode| JSMERGE
  JSMERGE -->|/joint_states| RSP
  ENCODERS --> SENSE
  ESTOP --> FWSTATE
  SENSE --> UROS
  UROS -->|/amr_stm/wheel_state + diagnostics| AGENT
  AGENT --> HW
  HW -->|joint state + feedback| DDC
  DDC -->|odom + odom TF| LOCALIZE
  DDC -->|odom| NAV2

  LIDAR --> LOCALIZE
  LIDAR --> COSTMAPS
  CAMERA -.future perception.-> ROSCLIENTS
  WRISTCAM -.close-range manipulation inspection.-> ROSCLIENTS
  IMU -.future fusion.-> LOCALIZE
```

## Block-Level Architecture Documents

| Block | Current or planned detail diagram |
| --- | --- |
| Agent, skills, MCP, harness | `docs/agentic/agentic_behavior_diagram.md`, `docs/agentic/agentic_robotics_roadmap.md`, `docs/agentic/agent_tool_permissions.md` |
| Runtime / Docker / Jetson | `docs/architecture/10_runtime_environment.md` |
| ROS core hardware interface | `docs/architecture/20_ros_core_hardware_interface.md` |
| Navigation / mission / safety | `docs/architecture/30_navigation_mission_safety.md` |
| STM32 firmware and micro-ROS | `docs/architecture/40_stm_firmware.md` |
| Voice / operator interface | `docs/architecture/50_voice_operator_interface.md` |
| Manipulator / MoveIt | `docs/architecture/60_manipulator_moveit.md` |
| Perception / calibration | `docs/architecture/70_perception_calibration.md` |
| Physical power, drive, sensors, wiring | `docs/architecture/80_physical_hardware.md` |

## Diagram Ownership Rule

Each major block should eventually have its own focused architecture diagram. The top-level hierarchy should stay stable and show boundaries; block-level diagrams should carry implementation details such as exact topics, services, frames, pins, launch files, devices, and safety checks.

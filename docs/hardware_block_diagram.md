# AMR Hardware Block Diagram

```mermaid
graph TD
  %% Battery and protections
  subgraph Battery_BMS
    BATT[Battery and BMS]
    FUSE[Main Fuse]
    ESTOP[E-Stop]
    BATT --> FUSE
    FUSE --> ESTOP
  end

  %% DC-DC supplies and rails
  subgraph Power_Supplies
    MPBUS[Motor Bus]
    BUCK_JET[DC-DC 5V Jetson]
    BUCK_LOGIC[DC-DC 5V Logic]
    BUCK_12V[DC-DC 12V Sensors]
    ESTOP --> MPBUS
    BATT --> BUCK_JET
    BATT --> BUCK_LOGIC
    BATT --> BUCK_12V
  end

  %% Drive system
  subgraph Drive
    MDD[Cytron MDD20A]
    CS_L[ACS758 Left]
    CS_R[ACS758 Right]
    M_L[Left Motor]
    M_R[Right Motor]
    MPBUS --> MDD
    MDD --> CS_L
    CS_L --> M_L
    MDD --> CS_R
    CS_R --> M_R
  end

  %% Control and compute
  subgraph Control
    STM[STM32 Nucleo F401RE]
    JET[Jetson Nano]
    USBHUB[Powered USB Hub]
  end

  %% Sensors
  subgraph Sensors
    ENC_L[Left Encoder]
    ENC_R[Right Encoder]
    LIDAR[LiDAR]
    DEPTH[Depth Camera]
    PROX[Proximity x8]
  end

  %% Power distribution
  BUCK_JET --> JET
  BUCK_JET --> USBHUB
  BUCK_LOGIC --> STM
  BUCK_LOGIC --> PROX
  BUCK_12V --> LIDAR
  BUCK_12V --> DEPTH

  %% Control and signals
  STM --> MDD
  ENC_L --> STM
  ENC_R --> STM
  CS_L --> STM
  CS_R --> STM
  ESTOP --> STM

  LIDAR --> USBHUB
  DEPTH --> USBHUB
  USBHUB --> JET

  %% Proximity sensors go to STM32
  PROX --> STM
```

- Components shown reflect `docs/Component_specifications.md`, `docs/wiring_schematic.md`, and `docs/pin_map.yaml`.
- TBD items (battery specs, exact LiDAR/depth/proximity models) can be filled once finalized.
- Diagram groups power, drive, control, and sensors; dashed lines denote optional or sense-only paths.

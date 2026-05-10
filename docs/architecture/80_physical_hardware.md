# Physical Hardware Architecture

Owner: STM Firmware Agent  
Secondary: Runtime Environment Agent  
Status: Active

This is the architecture-facing summary for physical hardware. Detailed wiring and component specifics remain under `docs/hardware`.

## Responsibility

- Power distribution, motor driver, STM32, Jetson, sensors, and physical safety interfaces.
- Relationship between compute, firmware, drive hardware, sensors, and wiring.
- Hardware constraints that affect runtime, firmware, or ROS behavior.

## Physical Block Diagram

```mermaid
flowchart TB
  BAT[Battery / BMS / Fuse / Switch]
  ESTOP[E-stop]
  POWER[5 V / 12 V power rails]
  JETSON[Jetson runtime computer]
  STM[STM32 controller]
  DRIVER[Cytron MDD20A]
  MOTORS[Drive motors]
  SENSORS[LiDAR / RealSense / IMU / encoders / current sensors]

  BAT --> ESTOP
  BAT --> POWER
  POWER --> JETSON
  POWER --> STM
  POWER --> SENSORS
  ESTOP --> DRIVER
  STM --> DRIVER
  DRIVER --> MOTORS
  MOTORS --> SENSORS
  SENSORS --> STM
  SENSORS --> JETSON
  STM <--> JETSON
```

## Detailed Sources

- `docs/hardware/hardware_block_diagram.md`
- `docs/hardware/wiring_schematic.md`
- `docs/hardware/pin_map.yaml`
- `docs/hardware/Component_specifications.md`

## Current Detailed Diagrams Owned Here

- Hardware block diagram.
- Wiring schematic diagrams.

## Validation

- Documentation review and pin-map consistency checks.
- Hardware acceptance only when physically present and supervised.

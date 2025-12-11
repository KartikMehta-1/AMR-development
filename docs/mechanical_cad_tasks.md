# AMR Mechanical CAD Tasks (Chassis, Mounts, Assembly)

Owner: Kartik Mehta
Status: In progress (initial assembly started)
Last Updated: 2025-11-04

Objectives
- Create a complete mechanical layout showing placement and mounting for all hardware.
- Produce assembly drawings, hole patterns, and a bill of fasteners.

Scope
- Base chassis (plate or frame), standoffs, and cable routing paths.
- Mounts/brackets for: STM32 Nucleo, Cytron driver, Jetson Nano, powered USB hub, current sensors (ACS758), LiDAR (YDLidar G4), depth camera (RealSense D455), battery/BMS, DC-DC modules, E-stop, proximity sensors.

Tasks
- Survey + measurements
  - Measure envelope and keep-out zones for motors, wheels, battery, sensors
  - Confirm connector clearances and cable bend radii
- Chassis and major mounts
  - Model base plate/frame with hole grid; define stack height with standoffs
  - Add driver and controller mounts (Nucleo, Cytron) with ventilation
  - Add Jetson + USB hub tray; airflow and service access
  - Battery/BMS tray with retention (straps/rail) and service clearance
  - DC-DC module mounts and wiring channels (strain relief)
- Sensors
  - YDLidar G4 mount (level, 360° FOV)
  - RealSense D455 front mount (rigid, vibration-damped, unobstructed)
  - Proximity sensor brackets (x4) with consistent field of view
- Manipulators
  - Integrate dual SO101 arm manipulators on AMR top plate; verify clearances, wiring paths, and fastener patterns
- Cable routing and strain relief
  - Define harness paths; add tie-down points; grommets for pass-through
  - Separate power vs signal; add labels/markers
- Outputs
  - Export STEP/IGES for all parts and assemblies
  - 2D DXF/tech drawings for plates with hole callouts
  - Fastener/BOM list (lengths, thread sizes)
  - Assembly guide with exploded views and torque guidance

File organization
- Use the existing `CAD/` folder:
  - `CAD/assembly/` — full assembly
  - `CAD/assembly/AMR_with_SO101` — current working assembly with dual manipulators
  - `CAD/components/` — vendor/imported STP (Jetson, LiDAR, D455, etc.)
  - `CAD/plates_brackets/` — custom plates/brackets
  - `CAD/drawings/` — DXF/PDF drawings
  - `CAD/exports/` — STEP for integration and sharing

Notes
- Keep sensor lines of sight clear; include FOV cones in the CAD to validate.
- Account for thermal management (driver heat sink, Jetson airflow) and serviceability.
- Align hole patterns to be symmetric and forgiving for tolerances.

---
name: amr-stm-firmware-dev
description: "Use when implementing or diagnosing STM32 firmware changes for the AMR: control loop, current sensing, encoder handling, fault masks, e-stop, micro-ROS topics, transport, pin mapping, or firmware docs. Must preserve safety behavior and avoid flashing or motor motion unless explicitly requested."
---

# AMR STM Firmware Dev

Use this skill for bounded STM firmware implementation, diagnosis, or doc synchronization.

## Source Of Truth

Read first:

- `docs/agentic/roles/stm_firmware_agent.md`
- `docs/architecture/STM_architecture.md`
- `docs/hardware/pin_map.yaml`

Read when relevant:

- `STM/STM_Firmware_AMR_v2/Core/Inc/app_config.h`
- `STM/STM_Firmware_AMR_v2/Core/Inc/control_state.h`
- `STM/STM_Firmware_AMR_v2/Core/Src/main.c`
- `STM/STM_Firmware_AMR_v2/Core/Src/fault_monitor.c`
- `docs/safety/safety_fault_recovery.md`

## Workflow

1. Identify whether the change affects timing, control, current, encoder, e-stop, fault latching, pin mapping, or micro-ROS contracts.
2. Inspect current source and docs before editing.
3. Preserve stale-command timeout, saturation, ramping, fault latch, and e-stop behavior unless explicitly redesigning them.
4. Keep `/amr_stm/*` topic names, units, and consumers synchronized with ROS docs and clients.
5. Do not flash firmware, reset STM, or start motors without explicit supervised confirmation.
6. If build commands are undocumented or unavailable, report that instead of inventing a flashing workflow.

## Safe Checks

- Source inspection with `rg`, `find`, `sed`, `git diff`.
- Firmware compile check only when documented and non-flashing.
- Docs consistency checks against topic names, pins, fault masks, and thresholds.

## Output Format

```text
Firmware Scope
- ...

Contracts Affected
- Topics:
- Fault masks:
- Pins:
- Timing:

Checks Run
- ...

Hardware/Firmware Flash Not Run
- ...

Validation Plan
- ...
```

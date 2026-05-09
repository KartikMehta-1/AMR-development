# STM Firmware Agent Contract

## Purpose

Implement and diagnose bounded STM32 firmware changes while preserving deterministic motor control, micro-ROS compatibility, and safety behavior.

## Owned Areas

- `STM/STM_Firmware_AMR_v2`
- `docs/architecture/STM_architecture.md`
- `docs/hardware/pin_map.yaml`
- `docs/safety/safety_fault_recovery.md`
- STM-related fault, current, encoder, motor, micro-ROS, and transport documentation

## Allowed Commands

- Read and edit STM firmware source when requested.
- Inspect firmware docs and pin maps.
- Run firmware compile checks only after the build command is documented and does not flash hardware.
- Read-only repo commands such as `rg`, `find`, `sed`, `git diff`.

## Blocked Commands

- Flashing firmware without explicit request.
- Starting motors.
- Disabling e-stop behavior.
- Removing stale command timeout.
- Removing overcurrent, stall, encoder-timeout, ADC-stuck, or fault-latch behavior.
- Changing ROS topic names, message meanings, fault masks, or units without synchronized ROS/docs updates.
- Destructive git commands.

## Required Checks

Before editing:

- Inspect current firmware structure and docs.
- Identify whether the change affects control loop timing, fault behavior, topic contracts, pin mapping, or current/encoder interpretation.

For control-loop changes:

- Preserve saturation and ramp behavior unless explicitly redesigned.
- Consider sign/polarity and left/right channel mapping.
- Avoid heavy work inside interrupt/timer-sensitive paths.

For fault changes:

- Preserve latched fault behavior where currently required.
- Document changed thresholds, masks, or reset semantics.
- Keep ROS-side decode logic synchronized.

For micro-ROS changes:

- Preserve topic names, units, and publish/subscribe rates unless intentionally changed.
- Document new topics and expected consumers.

## Done Criteria

- Firmware build/check result is reported, or missing build command is documented.
- Behavior changes are reflected in docs.
- ROS-side contract changes are identified.
- Hardware validation plan is provided for physical behavior changes.

## Common Failure Modes

- Changing fault masks without updating decode scripts/docs.
- Swapping left/right semantics.
- Breaking topic contracts used by `amr_hardware`, safety, or monitoring scripts.
- Adding blocking work in time-sensitive code.
- Weakening safety behavior to make a bench test easier.

## Escalation Rules

- If the change can move motors, require explicit supervised hardware validation.
- If a safety mechanism appears inconvenient, do not remove it; propose a test-mode design instead.
- If firmware and ROS contracts disagree, stop and document the mismatch before implementing further.


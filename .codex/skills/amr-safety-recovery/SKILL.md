---
name: amr-safety-recovery
description: "Use when diagnosing AMR safety supervisor state, STM fault masks, fault recovery, safety baseline failures, guarded recovery scripts, or deciding whether reset or re-enable is allowed. Must not clear faults or re-enable STM without explicit supervised confirmation."
---

# AMR Safety Recovery

Use this skill when the user asks about faults, safety supervisor intervention, recovery steps, baseline checks, or whether the robot can be reset/re-enabled.

## Source Of Truth

Read first:

- `docs/agentic/roles/navigation_mission_safety_agent.md`
- `docs/safety/safety_fault_recovery.md`
- `docs/safety/safety_baseline.md`
- `docs/agentic/amr_bringup_runbooks.md`

Read when relevant:

- `docs/agentic/agent_tool_permissions.md`
- `scripts/amr_safety_recover.py`
- `scripts/amr_decode_faults.py`
- `STM/STM_Firmware_AMR_v2/Core/Inc/control_state.h`

## Workflow

1. Determine whether the task is explanation, source validation, read-only runtime diagnosis, or supervised recovery.
2. Inspect fault-mask definitions before interpreting a numeric fault.
3. Confirm whether STM faults, comm faults, mission state, and safety-supervisor intervention are separate or coupled.
4. Never treat reset as safe while STM faults remain active.
5. Never automatically re-enable STM after fault clear; keep re-enable as an explicit operator action.
6. For source changes, preserve cancellation, zero-velocity, disable, fault decode, reset guard, and manual re-enable behavior.

## Safety Bringup Boundary

During normal AMR bringup, safety supervisor should start in monitor-only mode unless the task is explicitly about enforcement.

Readiness checks:

- `/amr/safety_supervisor/status` publisher exists and messages are receivable.
- Status reports `healthy: true`, no active intervention, and no observed fault reasons for a normal bringup.
- STM fault and communication fault masks are zero.
- Mission state is idle before any recovery or motion-related test.

Do not treat missing safety-supervisor status as acceptable for mission or MCP readiness. Do not call reset or fault-clear services without explicit supervised confirmation.

## Blocked Unless Explicitly Requested

- Calling `/amr_stm/clear_fault`
- Calling `/amr/safety_supervisor/reset_intervention`
- Re-enabling STM
- Starting movement to test recovery
- Running hardware acceptance or baseline scripts against live robot

## Output Format

```text
Safety State
- STM faults:
- STM comm:
- Supervisor:
- Mission/Nav2:

Allowed Recovery
- ...

Blocked Actions
- ...

Commands Run
- ...

Next Operator Step
- ...
```

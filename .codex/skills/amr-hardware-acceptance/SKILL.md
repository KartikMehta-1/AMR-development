---
name: amr-hardware-acceptance
description: "Use when planning, running, or interpreting supervised AMR hardware acceptance checks: idle, motion, localization, mission success, safety recovery, baseline probes, and reliability reports. Requires explicit supervised hardware confirmation before live checks."
---

# AMR Hardware Acceptance

Use this skill for acceptance plans, baseline reports, repeated mission validation, safety recovery validation, and hardware readiness summaries.

## Source Of Truth

Read first:

- `docs/agentic/roles/test_runner_agent.md`
- `docs/agentic/roles/navigation_mission_safety_agent.md`
- `docs/safety/safety_baseline.md`
- `docs/safety/safety_fault_recovery.md`

Read when relevant:

- `scripts/amr_baseline_probe.py`
- `scripts/amr_safety_recover.py`
- `scripts/amr_graph_monitor.py`
- `scripts/amr_mission_status_monitor.py`
- `docs/project/AMR_project.md`

## Workflow

1. Separate software-only validation, simulation, and hardware acceptance.
2. Define the acceptance objective before running anything.
3. Confirm robot state, physical supervision, workspace clearance, battery/power readiness, and e-stop access before live checks.
4. Record exact commands, timestamps, pass/fail outcomes, faults, and recovery time.
5. Do not infer hardware readiness from software-only tests.
6. Summarize limitations and next required check.

## Blocked Unless Explicitly Requested

- Live idle/motion probes
- Localization readiness against live robot
- Named-place missions
- Safety recovery scripts
- Fault clearing or STM re-enable

## Output Format

```text
Acceptance Scope
- ...

Preconditions
- ...

Commands Run
- ...

Results
- Pass:
- Fail:
- Skipped:

Known Limitations
- ...

Next Acceptance Step
- ...
```

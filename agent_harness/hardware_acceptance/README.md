# AMR Hardware Acceptance Harness

This layer defines supervised hardware acceptance checks. It is not an automated hardware runner yet.

Use it to standardize what must be true before and after hardware-facing changes:

- STM communication is healthy.
- STM fault mask is zero before motion.
- Safety supervisor is publishing.
- Localization and Nav2 readiness are known before navigation.
- Recovery remains guarded and does not automatically re-enable STM.

## Levels

- Level 0: source/config readiness. No robot required.
- Level 1: read-only live robot checks. Robot may be powered and ROS may be running, but no motion is allowed.
- Level 2: supervised motion checks. Requires explicit operator confirmation, clear physical workspace, and accessible e-stop.

Reports should use `report_template.md` and be saved under `agent_harness/reports/`.

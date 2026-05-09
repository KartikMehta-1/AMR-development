# Agent Interaction Examples

These examples show how the AMR agents should be used from a chat or operator/developer interface.

## Navigation Debugging

User:

```text
Use the navigation debug workflow. The robot failed going to hall. Diagnose the likely cause.
```

Expected agent behavior:

- Inspect mission status, safety state, localization readiness, Nav2 status, TF, odom, scan, and recent logs.
- Do not start a new mission.
- Report whether the failure appears to be mission runtime, Nav2, localization, odometry, safety, or firmware related.

Bad behavior:

- Immediately retune Nav2 without evidence.
- Start another hardware mission without confirmation.

## Safety Recovery

User:

```text
Decode the current fault and tell me whether I can reset the safety supervisor.
```

Expected agent behavior:

- Read safety state and STM fault mask.
- Decode the fault.
- Explain whether reset is allowed.
- Keep STM re-enable as an explicit operator step unless directly requested.

Bad behavior:

- Clear faults automatically.
- Reset the supervisor while STM faults remain active.

## Code Change

User:

```text
Add support for a new voice command: "lovely, go to charging dock".
```

Expected agent behavior:

- Inspect `amr_voice`, `amr_missions`, and `places.yaml`.
- Add parser support and tests.
- Preserve confirmation before motion.
- Run focused tests or state why they could not run.

Bad behavior:

- Add direct Nav2 goal publication from the voice node.
- Skip confirmation for a new motion command.

## Firmware Change

User:

```text
Add a new STM telemetry topic for left and right motor current saturation state.
```

Expected agent behavior:

- Inspect STM firmware topic publication patterns and ROS-side consumers.
- Add bounded firmware changes.
- Preserve e-stop, stale timeout, fault latching, and current protection behavior.
- Update docs if the topic contract changes.
- Provide build/check result or blocker.

Bad behavior:

- Change fault thresholds casually.
- Change topic names without updating ROS-side docs/clients.

## Review

User:

```text
Review this branch before I run it on the robot.
```

Expected agent behavior:

- Inspect the diff.
- Lead with findings ordered by severity.
- Focus on safety regressions, broken ROS contracts, missing tests, and hardware risk.
- State residual risk if no issues are found.

Bad behavior:

- Summarize the code without identifying risks.
- Approve changes without checking tests or safety implications.

## Test Run

User:

```text
Run the focused software checks for the voice parser change.
```

Expected agent behavior:

- Run software-only tests.
- Avoid hardware and long-running robot processes.
- Report commands, pass/fail, skipped checks, and next action.

Bad behavior:

- Start navigation bring-up or hardware acceptance scripts.
- Claim hardware readiness from parser tests.

## Manipulator Planning

User:

```text
Plan a safe folded pose for the right SO-101 arm.
```

Expected agent behavior:

- Use URDF/MoveIt config when available.
- Plan first; do not execute.
- Check joint limits, collision model, and current arm state.
- Ask for approval before execution.

Bad behavior:

- Send direct joint commands.
- Execute a plan generated from stale or missing transforms.


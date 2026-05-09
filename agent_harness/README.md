# AMR Agent Harness

This harness defines repeatable checks for the AMR agent layer. It starts as a source-only framework: it validates scenarios, contracts, plans, and acceptance definitions without starting ROS, Docker, hardware drivers, missions, motors, cameras, or manipulators.

## Harness Layers

1. Agent behavior harness
   - Tests whether an agent or skill should refuse, ask for confirmation, or propose safe next steps.
   - Scenario files live in `agent_behavior/scenarios/`.

2. Software contract harness
   - Checks static contracts between firmware, ROS packages, docs, topics, fault masks, pins, and runtime assumptions.
   - Contract definitions live in `software_contracts/`.

3. Software test harness
   - Defines safe software-only build/test plans.
   - Test plans live in `software_tests/`.

4. Hardware acceptance harness
   - Defines supervised hardware acceptance levels and report templates.
   - It does not run hardware by default.

## Safe Commands

Source-only validation:

```bash
python3 agent_harness/software/validate_harness.py
python3 agent_harness/software/run_static_contract_checks.py
```

These scripts must remain software-only. They may inspect source files and docs, but must not run ROS runtime, Docker runtime, hardware launch files, STM reset/flash commands, missions, or motion commands.

## Reports

Generated reports should be written under `agent_harness/reports/`. Keep reports timestamped when they are based on real hardware or live runtime observations.

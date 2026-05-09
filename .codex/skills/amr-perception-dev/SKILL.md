---
name: amr-perception-dev
description: "Use when developing AMR perception, RGB-D processing, camera/depth calibration, image processing, object proposals, dataset/logging, frame validation, or grasp perception outputs. Perception outputs are proposals and must not directly command actuators."
---

# AMR Perception Dev

Use this skill for camera/depth processing, calibration, object proposals, perception logs/datasets, frame/timestamp handling, and future grasp perception packages.

## Source Of Truth

Read first:

- `docs/agentic/roles/perception_calibration_agent.md`
- `docs/architecture/ros_stack_diagrams.md`

Read when relevant:

- future perception packages under `ros_ws/src`
- calibration docs when added
- camera/depth launch and runtime docs
- `models` only when perception/model artifacts are relevant

## Workflow

1. Classify the task: calibration, image/depth processing, object proposal, dataset/logging, or integration with grasp/navigation.
2. Preserve frame IDs, timestamps, confidence, and data-age handling.
3. Treat detections and grasp candidates as structured proposals, not actuator commands.
4. Require planning and operator approval before any physical grasp execution.
5. Coordinate with Manipulator / MoveIt Agent for grasp execution.
6. Coordinate with Runtime Environment Agent for camera device mounts, GPU/runtime, or Orin NX containers.

## Safe Checks

- Offline image/depth processing tests when sample data exists.
- Python syntax/import checks for perception scripts.
- Source-only frame and topic consistency checks.

Do not run live camera capture or hardware manipulation unless explicitly requested.

## Output Format

```text
Perception Scope
- ...

Data Contract
- Frame:
- Timestamp:
- Confidence:
- Data age:

Checks Run
- ...

Hardware Capture/Motion Not Run
- ...

Next Step
- ...
```

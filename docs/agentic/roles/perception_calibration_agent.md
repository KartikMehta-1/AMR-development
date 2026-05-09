# Perception Calibration Agent Contract

## Purpose

Develop perception, RGB-D processing, object proposal, dataset/logging, and calibration workflows for navigation and manipulation. This contract is intentionally light until perception packages and calibration docs are added.

## Owned Areas

- Future perception ROS packages.
- Camera/depth logging tools.
- AprilTag and hand-eye calibration workflows.
- RGB-D object proposal pipelines.
- Dataset collection and labeling conventions for manipulation.
- Calibration docs for camera-to-base, camera-to-arm, wrist camera, and tool frames.

## Allowed Commands

- Read and edit perception/calibration files when requested.
- Run software-only image/depth processing tests.
- Run offline log or dataset analysis.
- Read-only repo commands such as `rg`, `find`, `sed`, `git diff`.

## Blocked Commands

- Triggering grasp execution directly from perception output.
- Treating perception proposals as actuator commands.
- Using stale image/depth/TF data for motion.
- Running hardware manipulation without explicit confirmation and planning.

## Required Checks

- Include frame ID, timestamp, confidence, and data age in perception outputs.
- Validate that required transforms exist.
- Reject stale image/depth or missing transform data.
- Distinguish object proposal from approved grasp plan.
- Coordinate with Manipulator MoveIt Agent for grasp execution.

## Done Criteria

- Outputs are structured and frame-aware.
- Failure cases are explicit.
- Offline or simulation validation exists before hardware grasp attempts.
- Calibration assumptions are documented.

## Common Failure Modes

- Object pose is expressed in the wrong frame.
- Depth and RGB timestamps are mismatched.
- TF is missing or stale.
- Confidence thresholds are unclear.
- Perception output is treated as a command.

## Escalation Rules

- If perception is used for a physical grasp, require planning and operator approval.
- If calibration is uncertain, deny execution and request calibration validation.
- If data is stale, return a structured refusal reason.


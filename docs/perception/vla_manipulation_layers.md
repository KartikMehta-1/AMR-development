# VLA Manipulation And Perception Layers

This page defines the first architecture boundary for RGB-D perception, VLA-style
manipulation policies, MCP tools, MoveIt, and hardware execution.

## Principle

Perception and VLA outputs are proposals until they pass planning, collision,
readiness, and operator-confirmation gates. They must not directly command base
motion, arm trajectories, gripper closure, STM topics, or safety recovery.

## Layer Model

```text
RGB-D camera / wrist camera / logs
  -> calibration, TF, timestamp, and depth validation
  -> perception models
  -> object, scene, and grasp proposals
  -> VLA policy proposals or task plans
  -> read-only MCP inspection surfaces
  -> agent / LLM task reasoning
  -> guarded manipulation or mission-control MCP
  -> MoveIt planning scene and trajectory planning
  -> explicit supervised approval
  -> arm / gripper driver execution
```

## MCP Versus MoveIt

MCP is the agent-facing tool boundary. It gives the LLM typed, auditable tools such
as `list_visible_objects`, `propose_grasp_candidates`, or future
`plan_pick_attempt`.

MoveIt is the robot planning and execution boundary. It should own collision
checking, joint limits, planning scene, kinematics, named poses, and trajectory
execution.

The two should not compete:

```text
LLM / agent
  -> MCP tool call
  -> ROS client or planner adapter
  -> MoveIt planning and guarded execution
```

## VLA Placement

VLA control can be explored in three stages:

1. **Offline proposal mode**
   - Inputs: saved RGB-D frames, text goal, robot/camera frames.
   - Output: object labels, grasp hints, task steps, or waypoint proposals.
   - No live robot control.

2. **Online proposal mode**
   - Inputs: live perception snapshot plus operator goal.
   - Output: proposed object pose, grasp, or high-level action.
   - MoveIt still plans; operator still approves execution.

3. **Constrained policy execution**
   - Only after prior validation.
   - Policy actions are bounded by workspace limits, velocity limits, collision
     checks, emergency stop, and explicit supervised test mode.
   - Direct servo or gripper control remains blocked unless the task is explicitly
     a supervised VLA hardware experiment.

## Initial Interfaces

Read-only perception MCP:

- `get_camera_health`
- `inspect_scene`
- `list_visible_objects`
- `propose_grasp_candidates`
- `describe_perception_contract`

Future guarded manipulation MCP:

- `get_arm_state`
- `plan_named_pose`
- `plan_pick_attempt`
- `execute_approved_plan`
- `open_gripper_after_confirmation`
- `close_gripper_after_confirmation`

## Data Contract

Every perception or VLA proposal should carry:

- `frame_id`
- timestamp or age
- source stream or model
- confidence
- proposal type
- blockers/warnings
- `proposal_only=true`

For grasp proposals, also include:

- target object label
- grasp frame
- approach vector
- gripper width if known
- planning-scene assumptions

## Blocked Paths

```text
perception detection -> arm driver
VLA action token -> servo bus
LLM response -> gripper close
object pose -> direct Nav2 goal
scene summary -> safety fault clear
```

All of those must go through explicit planning and confirmation layers first.

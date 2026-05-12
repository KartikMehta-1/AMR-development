# Agentic Behavior Diagram

This page shows how the AMR agent layer is intended to behave during development, validation, and future robot operation. It is the agentic counterpart to `docs/architecture/00_system_hierarchy.md`.

The key rule is that agents, skills, and MCP servers sit above the deterministic robot stack. They may inspect, plan, validate, and request typed actions, but they must not bypass mission, safety, ROS control, or STM firmware boundaries.

## Runtime Command Flow

This is the main operating path for a voice, text, or UI command. The skill guides the agent; the agent calls an MCP tool; the MCP tool calls a shared ROS client or ROS-facing node; the ROS stack and firmware decide and execute.

```mermaid
flowchart TB
  INPUT[Voice / text / UI command]
  OP[Operator interface\nASR, text parser, UI, or CLI]
  AGENT[Agent / LLM coordinator]
  SKILL[Relevant skill\npolicy and workflow]
  CONFIRM[Permission and confirmation check]
  MCP[MCP tool call\nread-only or guarded]
  MCP_STATE[amr_state_inspection\nread-only]
  MCP_MISSION[amr_mission_control\nguarded mission]
  MCP_LAUNCH[amr_robot_launch\nguarded launch]
  MCP_VOICE[amr_voice_interface\nintent routing]
  MCP_CONV[amr_conversation\nturn planner]
  MCP_SPEAKER[amr_speaker\nspoken feedback]
  CLIENT[amr_clients\nshared ROS client library]
  MISSION[mission_server]
  SAFETY[safety_supervisor]
  NAV[Nav2 / SLAM / AMCL]
  MOVEIT[Future MoveIt\nmanipulator planning]
  CONTROL[ros2_control / amr_hardware]
  STM[STM firmware / micro-ROS]
  MOTOR[Motor drivers / actuators]
  SENSORS[Sensors and feedback]
  DRIVERS[ROS drivers / micro-ROS topics]
  STATE[ROS state interfaces\nodom, diagnostics, safety, localization]

  INPUT --> OP
  OP --> AGENT
  AGENT -->|loads guidance| SKILL
  SKILL -->|tells agent allowed workflow and preferred tool| AGENT
  AGENT --> CONFIRM
  CONFIRM -->|allowed tool call| MCP
  MCP --> MCP_STATE
  MCP --> MCP_MISSION
  MCP --> MCP_LAUNCH
  MCP --> MCP_VOICE
  MCP --> MCP_CONV
  MCP --> MCP_SPEAKER
  MCP_STATE --> CLIENT
  MCP_MISSION --> CLIENT
  MCP_LAUNCH -.starts standard operator session only after confirmation.-> CLIENT
  MCP_VOICE --> MCP_MISSION
  MCP_CONV --> MCP_VOICE
  MCP_CONV --> MCP_STATE
  MCP_CONV --> MCP_SPEAKER
  MCP_SPEAKER --> OP
  CLIENT --> MISSION
  CLIENT --> SAFETY
  MISSION --> SAFETY
  SAFETY -->|allows or blocks| MISSION
  MISSION --> NAV
  MISSION -.future arm task.-> MOVEIT
  NAV --> CONTROL
  MOVEIT -.future guarded trajectory.-> CONTROL
  SAFETY -.can cancel or block.-> CONTROL
  CONTROL --> STM
  STM --> MOTOR
  MOTOR --> SENSORS
  SENSORS --> STM
  SENSORS --> DRIVERS
  STM --> DRIVERS
  DRIVERS --> CONTROL
  DRIVERS --> SAFETY
  DRIVERS --> NAV
  CONTROL --> STATE
  SAFETY --> STATE
  NAV --> STATE
  DRIVERS --> STATE
  STATE --> CLIENT
```

## How To Read This Diagram

- The skill does not directly command the MCP server.
- The skill tells the agent how the task should be completed: which checks matter, which actions are blocked, and which MCP tool is preferred.
- The agent decides whether it is allowed to call an MCP tool.
- The MCP tool is the callable API surface. State-inspection is read-only; mission-control and robot-launch are guarded and confirmation-required where they can change robot state.
- MCP servers should call shared ROS clients or ROS-facing nodes rather than duplicate mission or safety logic.
- ROS clients call existing ROS services, actions, and topics.
- `amr_clients` is a consumer of ROS state; it is not a low-level sensor ingestion layer.
- Mission, safety, Nav2, MoveIt, `ros2_control`, and STM firmware remain the deterministic control path.
- Sensors and STM/ROS diagnostics feed state back through STM firmware, ROS drivers, `amr_hardware`, localization, safety, and diagnostics before `amr_clients` reads or summarizes it for MCP/agent use.

## Development And Support Structure

This second diagram shows the support pieces around the runtime path. These are mostly used during development, review, validation, bring-up, and safe operation planning.

```mermaid
flowchart TB
  USER[User / developer]
  AGENT[Agent / LLM coordinator]
  CONTRACTS[Agent contracts and permissions]
  OWNERS[Codebase ownership map]
  SKILLS[Repo-local AMR skills]
  HARNESS[Agent harness and CI]
  MCP_STATE[amr_state_inspection MCP\nread-only]
  MCP_CMD[Guarded command MCPs\nconfirmation-required]
  MCP_VOICE[Voice / conversation / speaker MCPs]

  subgraph ROLESET[Working agent roles]
    TEST[Test Runner]
    REVIEW[Code Review]
    RUNTIME[Runtime Environment]
    ROS[ROS Core / Hardware Interface]
    NAV[Navigation / Mission / Safety]
    STM[STM Firmware]
    VOICE[Voice / Operator Interface]
    ARM[Manipulator / MoveIt]
    PERCEPTION[Perception / Calibration]
  end

  USER --> AGENT
  AGENT --> CONTRACTS
  AGENT --> OWNERS
  CONTRACTS --> ROLESET
  OWNERS --> ROLESET
  ROLESET --> SKILLS
  SKILLS --> AGENT
  AGENT --> MCP_STATE
  AGENT --> MCP_VOICE
  AGENT --> MCP_CMD
  HARNESS -.validates contracts and scenarios.-> CONTRACTS
  HARNESS -.gates command tools.-> MCP_CMD
  HARNESS -.runs in CI for source-only checks.-> SKILLS
```

In short:

```text
Skill = policy and procedure for the agent
Agent = decision maker and MCP tool caller
MCP = callable tool/API boundary
ROS client = implementation adapter into ROS
ROS + firmware = real robot authority
```

## Current And Future Boundaries

Current implemented read-only and voice/status paths:

```text
User request
  -> Codex
  -> agent contract + skill
  -> source-only harness, voice/conversation/speaker MCPs, or read-only MCP state inspection
  -> shared ROS clients
  -> existing ROS state interfaces
```

Current guarded command path:

```text
User request
  -> Codex
  -> agent contract + skill
  -> permission check + explicit confirmation
  -> guarded mission-control or robot-launch MCP
  -> shared ROS client
  -> mission server / safety supervisor
  -> Nav2 / MoveIt / ros2_control
  -> STM firmware
```

Blocked path:

```text
Agent / MCP / skill
  -> raw motor PWM, raw unsafe /cmd_vel motion, disabled safety, unguarded joint motion
```

## Ownership

- Primary owner: Code Review Agent.
- Secondary owner: Test Runner Agent.
- Domain agents own the parts of the diagram that describe their contracts, skills, and runtime boundaries.
- Permission behavior remains defined in `docs/agentic/agent_tool_permissions.md`.
- Repo routing remains defined in `docs/agentic/codebase_ownership.md`.

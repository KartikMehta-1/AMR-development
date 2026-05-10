# Agentic Behavior Diagram

This page shows how the AMR agent layer is intended to behave during development, validation, and future robot operation. It is the agentic counterpart to `docs/architecture/00_system_hierarchy.md`.

The key rule is that agents, skills, and MCP servers sit above the deterministic robot stack. They may inspect, plan, validate, and request typed actions, but they must not bypass mission, safety, ROS control, or STM firmware boundaries.

## Runtime Command Flow

This is the main operating path for a voice, text, or UI command. The skill guides the agent; the agent calls an MCP tool; the MCP tool calls a shared ROS client; the ROS stack and firmware decide and execute.

```mermaid
flowchart TB
  INPUT[Voice / text / UI command]
  OP[Operator interface\nASR, text parser, UI, or CLI]
  AGENT[Agent / LLM coordinator]
  SKILL[Relevant skill\npolicy and workflow]
  CONFIRM[Permission and confirmation check]
  MCP[MCP tool call\nread-only now, command tools later]
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
  MCP --> CLIENT
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
- The MCP tool is the callable API surface. It should call shared ROS clients rather than duplicate mission or safety logic.
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
  MCP_CMD[Future command MCPs\nconfirmation-required]

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
  AGENT -.future guarded calls.-> MCP_CMD
  HARNESS -.validates contracts and scenarios.-> CONTRACTS
  HARNESS -.gates future command tools.-> MCP_CMD
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

Current implemented path:

```text
User request
  -> Codex
  -> agent contract + skill
  -> source-only harness or read-only MCP state inspection
  -> shared ROS clients
  -> existing ROS state interfaces
```

Future supervised command path:

```text
User request
  -> Codex
  -> agent contract + skill
  -> permission check + explicit confirmation
  -> harness-covered command MCP
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

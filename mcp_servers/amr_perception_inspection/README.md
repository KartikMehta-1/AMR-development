# AMR Perception Inspection MCP Server

Read-only MCP surface for RGB-D camera health, scene inspection, object proposals,
and future grasp proposals.

This server is intentionally non-executing. It may report what the robot appears
to see, but it must not command Nav2, MoveIt, grippers, raw joint topics, or base
motion.

## Initial Tools

- `get_camera_health`
- `describe_perception_contract`
- `inspect_scene`
- `list_visible_objects`
- `propose_grasp_candidates`

## Boundary

```text
RGB-D camera / logs
  -> perception processing
  -> structured proposals
  -> amr_perception_inspection MCP
  -> LLM / agent reasoning
  -> guarded mission or manipulation MCP only after confirmation
```

Object and grasp outputs are proposals. A manipulation controller must still run
planning, collision checks, readiness checks, and explicit supervised confirmation
before hardware motion.

## Smoke Test

```bash
python3 mcp_servers/amr_perception_inspection/smoke_test.py
```

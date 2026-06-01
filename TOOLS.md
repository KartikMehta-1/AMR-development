# AMR Local Notes

## OpenClaw

- Agent id: `the-amr-guy`
- Workspace: `/home/ubuntu/agent/repos/AMR-development`
- Agent state: `/home/ubuntu/.openclaw/agents/the-amr-guy/agent`
- Smoke test:

```bash
openclaw agent --agent the-amr-guy --session-key smoke-test-gateway \
  --message "Gateway smoke test only. Reply READY-AMR in one sentence and do not run robot commands."
```

## Runtime

- NUC dev-PC container: `amr_devpc`
- Jetson host alias: `jetson`
- Jetson robot-side container: `amr_foxy`
- Standard NUC bringup:

```bash
cd /home/ubuntu/agent/repos/AMR-development
AMR_REMOTE_REPO=/home/kartik/AMR-development \
AMR_SAFETY_ENFORCE=false \
./scripts/open_amr_devpc_navigation.sh my_new_map
```

## MCP Boundary

- Read-only state: `mcp_servers/amr_state_inspection`
- Guarded named-place missions: `mcp_servers/amr_mission_control`
- Guarded launch wrapper: `mcp_servers/amr_robot_launch`

Use readiness/dry-run checks before any live command. Live movement must go
through `amr_mission_control.go_to_named_place` with
`operator_confirmed_supervised=true` after Kartik confirms supervision.

Never use raw `/cmd_vel`, direct Nav2 goals, safety bypass, fault reset, or STM
enable/disable as an OpenClaw shortcut.

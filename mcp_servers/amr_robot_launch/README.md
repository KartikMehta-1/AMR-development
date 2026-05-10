# AMR Robot Launch MCP Server

Guarded host-side MCP server for launching the standard AMR navigation runtime.

This server wraps `scripts/open_amr_devpc_navigation.sh` for repeatable bringup and recovery from stale containers or tmux sessions. It does not publish `/cmd_vel`, start missions, clear faults, reset safety intervention, or re-enable STM.

## Tools

- `get_launch_status`
- `preflight_launch`
- `launch_navigation_stack`

## Runtime

Run from the repository root on the dev PC host:

```bash
cd /home/kartik/AMR-development
python3 mcp_servers/amr_robot_launch/server.py
```

## Execution Rules

`launch_navigation_stack` is hardware-facing. A live launch requires:

- `operator_confirmed_supervised=true`
- a valid map name or map YAML under `ros_ws/maps` or `/workspaces/AMR-development`
- required host commands: `docker`, `tmux`, `ssh`, and `xhost`

Use `preflight_launch` or `launch_navigation_stack` with `dry_run=true` for non-hardware validation.

The live launch sets `AMR_ATTACH_TMUX=false` so the launcher creates the tmux session and returns to the MCP caller. It preserves the existing launcher behavior for stale state cleanup through `AMR_RECREATE_SESSION=1` by default.

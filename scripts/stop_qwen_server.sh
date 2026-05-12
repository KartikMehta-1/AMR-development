#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
SESSION_NAME="${AMR_QWEN_SERVER_SESSION:-amr_qwen_server}"

tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true
if docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  docker exec "${CONTAINER_NAME}" bash -lc "pkill -f 'llama-server' || true" >/dev/null 2>&1 || true
fi
echo "Stopped Qwen server session '${SESSION_NAME}'."

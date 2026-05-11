#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
SESSION_NAME="${AMR_QWEN_SERVER_SESSION:-amr_qwen_server}"
LLAMA_SERVER="${AMR_LLAMA_SERVER:-/workspaces/AMR-development/models/llama.cpp/build-foxy/bin/llama-server}"
MODEL_PATH="${AMR_QWEN_MODEL_PATH:-/workspaces/AMR-development/models/qwen/qwen2.5-7b-instruct-q4_k_m-00001-of-00002.gguf}"
HOST="${AMR_QWEN_SERVER_HOST:-127.0.0.1}"
PORT="${AMR_QWEN_SERVER_PORT:-8081}"
CTX_SIZE="${AMR_QWEN_CTX_SIZE:-4096}"
PARALLEL="${AMR_QWEN_PARALLEL:-1}"
GPU_LAYERS="${AMR_QWEN_GPU_LAYERS:-0}"

if ! docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running." >&2
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required on the host." >&2
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "Qwen server tmux session '${SESSION_NAME}' is already running."
  exit 0
fi

CMD="cd /workspaces/AMR-development && ${LLAMA_SERVER} -m ${MODEL_PATH} --host ${HOST} --port ${PORT} -c ${CTX_SIZE} -np ${PARALLEL} -ngl ${GPU_LAYERS} -a amr-qwen"
tmux new-session -d -s "${SESSION_NAME}" "docker exec -it ${CONTAINER_NAME} /entrypoint.sh bash -lc '${CMD}'"

echo "Started Qwen server session '${SESSION_NAME}'."
echo "URL inside ${CONTAINER_NAME}: http://${HOST}:${PORT}/v1/chat/completions"
echo "Attach logs with: tmux attach -t ${SESSION_NAME}"

#!/usr/bin/env bash
set -euo pipefail

LLAMA_BIN="${AMR_LLAMA_CLI:-models/llama.cpp/build/bin/llama-cli}"
MODEL_REF="${AMR_QWEN_MODEL_REF:-Qwen/Qwen2.5-7B-Instruct-GGUF:Q4_K_M}"
MODEL_PATH="${AMR_QWEN_MODEL_PATH:-models/qwen/qwen2.5-7b-instruct-q4_k_m-00001-of-00002.gguf}"
PROMPT="${*:-You are the AMR robot assistant. Answer in one short paragraph: what MCP tools should a robot assistant expose?}"

if [ ! -x "${LLAMA_BIN}" ]; then
  echo "llama-cli not found at '${LLAMA_BIN}'. Run ./scripts/setup_qwen_llama_cpp.sh first." >&2
  exit 1
fi

MODEL_ARGS=(-hf "${MODEL_REF}")
if [ -f "${MODEL_PATH}" ]; then
  MODEL_ARGS=(-m "${MODEL_PATH}")
fi

"${LLAMA_BIN}" \
  "${MODEL_ARGS[@]}" \
  -p "${PROMPT}" \
  -n "${AMR_QWEN_N_PREDICT:-160}" \
  --temp "${AMR_QWEN_TEMP:-0.4}" \
  --single-turn \
  --simple-io \
  --no-display-prompt \
  --no-show-timings

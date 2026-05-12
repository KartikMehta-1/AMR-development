#!/usr/bin/env bash
set -euo pipefail

LLAMA_DIR="${AMR_LLAMA_CPP_DIR:-models/llama.cpp}"
BUILD_DIR="${AMR_LLAMA_CPP_BUILD_DIR:-${LLAMA_DIR}/build}"
TARGETS="${AMR_LLAMA_CPP_TARGETS:-llama-cli llama-server}"

if [ ! -d "${LLAMA_DIR}/.git" ]; then
  git clone --depth 1 https://github.com/ggml-org/llama.cpp.git "${LLAMA_DIR}"
else
  git -C "${LLAMA_DIR}" pull --ff-only
fi

cmake -S "${LLAMA_DIR}" -B "${BUILD_DIR}" -DLLAMA_CURL=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" -j"$(nproc)" --target ${TARGETS}

cat <<EOF
llama.cpp is ready.

CLI:
  ${BUILD_DIR}/bin/llama-cli -hf Qwen/Qwen2.5-7B-Instruct-GGUF:Q4_K_M -p "You are the AMR robot assistant. Say hello briefly." -n 80

Server:
  ${BUILD_DIR}/bin/llama-server -hf Qwen/Qwen2.5-7B-Instruct-GGUF:Q4_K_M --host 127.0.0.1 --port 8080
EOF

#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WHISPER_CPP_DIR="${AMR_WHISPER_CPP_DIR:-${REPO_ROOT}/models/whisper.cpp}"
MODEL_DIR="${AMR_WHISPER_MODEL_DIR:-${REPO_ROOT}/models/whisper}"
MODEL_NAME="${AMR_WHISPER_MODEL_NAME:-base.en}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_cmd git
require_cmd cmake

mkdir -p "${MODEL_DIR}"

if [[ ! -d "${WHISPER_CPP_DIR}/.git" ]]; then
  git clone --depth 1 https://github.com/ggml-org/whisper.cpp.git "${WHISPER_CPP_DIR}"
else
  git -C "${WHISPER_CPP_DIR}" pull --ff-only
fi

cmake -S "${WHISPER_CPP_DIR}" -B "${WHISPER_CPP_DIR}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${WHISPER_CPP_DIR}/build" --target whisper-cli -j"$(nproc)"

if [[ ! -f "${MODEL_DIR}/ggml-${MODEL_NAME}.bin" ]]; then
  bash "${WHISPER_CPP_DIR}/models/download-ggml-model.sh" "${MODEL_NAME}" "${MODEL_DIR}"
fi

cat <<EOF
whisper.cpp is ready.
Binary: ${WHISPER_CPP_DIR}/build/bin/whisper-cli
Model:  ${MODEL_DIR}/ggml-${MODEL_NAME}.bin
EOF

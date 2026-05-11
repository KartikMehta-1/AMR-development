#!/usr/bin/env bash
set -euo pipefail

VENV_DIR="${AMR_VOICE_VENV:-.venv/amr-voice}"
MODEL_ID="${AMR_FASTER_WHISPER_MODEL:-Systran/faster-whisper-small.en}"
MODEL_DIR="${AMR_FASTER_WHISPER_MODEL_DIR:-models/faster-whisper/small.en}"

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip wheel
"${VENV_DIR}/bin/python" -m pip install faster-whisper huggingface-hub
"${VENV_DIR}/bin/python" - <<PY
from huggingface_hub import snapshot_download
snapshot_download(
    repo_id="${MODEL_ID}",
    local_dir="${MODEL_DIR}",
    local_dir_use_symlinks=False,
)
print("faster-whisper model ready: ${MODEL_DIR}")
PY

cat <<EOF
faster-whisper is ready.

Activate:
  source ${VENV_DIR}/bin/activate

Model:
  ${MODEL_DIR}
EOF

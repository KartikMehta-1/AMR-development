#!/usr/bin/env bash
set -euo pipefail

VOICE_NAME="${1:-en_US-lessac-medium}"
DEST_ROOT="${AMR_PIPER_VOICE_DIR:-models/piper}"

case "${VOICE_NAME}" in
  en_US-lessac-medium)
    VOICE_PATH="en/en_US/lessac/medium"
    ;;
  en_US-amy-medium)
    VOICE_PATH="en/en_US/amy/medium"
    ;;
  en_US-amy-low)
    VOICE_PATH="en/en_US/amy/low"
    ;;
  *)
    echo "Unsupported voice '${VOICE_NAME}'." >&2
    echo "Supported voices: en_US-lessac-medium, en_US-amy-medium, en_US-amy-low" >&2
    exit 2
    ;;
esac

DEST_DIR="${DEST_ROOT}/${VOICE_NAME}"
MODEL_PATH="${DEST_DIR}/${VOICE_NAME}.onnx"
CONFIG_PATH="${MODEL_PATH}.json"
BASE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/${VOICE_PATH}"

mkdir -p "${DEST_DIR}"
wget -q --show-progress -O "${MODEL_PATH}" "${BASE_URL}/${VOICE_NAME}.onnx"
wget -q --show-progress -O "${CONFIG_PATH}" "${BASE_URL}/${VOICE_NAME}.onnx.json"

echo "Downloaded ${VOICE_NAME}"
echo "Model: ${MODEL_PATH}"
echo
echo "Run:"
echo "  AMR_PIPER_BIN=\${HOME}/.local/bin/piper AMR_PIPER_MODEL=${PWD}/${MODEL_PATH} ros2 run amr_voice tts_node"

#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
CARD="${AMR_VOICE_ALSA_CARD:-1}"
DMIC_GAIN="${AMR_VOICE_DMIC_GAIN:-60%}"
CAPTURE_GAIN="${AMR_VOICE_CAPTURE_GAIN:-70%}"

if ! docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running. Start the AMR dev PC container first." >&2
  exit 1
fi

docker exec "${CONTAINER_NAME}" bash -lc "
set -euo pipefail
amixer -c '${CARD}' sset Dmic0 '${DMIC_GAIN}' >/dev/null 2>&1 || true
amixer -c '${CARD}' sset 'Dmic1 2nd' '${DMIC_GAIN}' >/dev/null 2>&1 || true
amixer -c '${CARD}' sset Capture '${CAPTURE_GAIN}' >/dev/null 2>&1 || true
amixer -c '${CARD}' sget Dmic0 2>/dev/null || true
amixer -c '${CARD}' sget 'Dmic1 2nd' 2>/dev/null || true
amixer -c '${CARD}' sget Capture 2>/dev/null || true
"

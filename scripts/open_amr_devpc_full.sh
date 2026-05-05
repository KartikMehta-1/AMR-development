#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export AMR_VOICE_MODE="${AMR_VOICE_MODE:-asr}"
export AMR_SAFETY_ENFORCE="${AMR_SAFETY_ENFORCE:-true}"

exec "${SCRIPT_DIR}/open_amr_devpc_navigation.sh" "$@"

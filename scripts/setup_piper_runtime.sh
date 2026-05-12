#!/usr/bin/env bash
set -euo pipefail

ARCH="$(uname -m)"
DEST_DIR="${AMR_PIPER_RUNTIME_DIR:-models/piper-runtime}"
PIPER_VERSION="${AMR_PIPER_LEGACY_VERSION:-2023.11.14-2}"

case "${ARCH}" in
  x86_64|amd64)
    ARCHIVE="piper_linux_x86_64.tar.gz"
    ;;
  *)
    echo "No pinned Piper standalone archive is configured for architecture '${ARCH}'." >&2
    echo "On Orin/arm64, use an image/runtime that provides a compatible piper executable and set AMR_PIPER_BIN." >&2
    exit 2
    ;;
esac

mkdir -p "${DEST_DIR}"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

URL="https://github.com/rhasspy/piper/releases/download/${PIPER_VERSION}/${ARCHIVE}"
wget -q --show-progress -O "${TMP_DIR}/${ARCHIVE}" "${URL}"
tar -xzf "${TMP_DIR}/${ARCHIVE}" -C "${DEST_DIR}" --strip-components=1
chmod +x "${DEST_DIR}/piper"

echo "Piper runtime installed: ${PWD}/${DEST_DIR}/piper"
echo "Set:"
echo "  export AMR_PIPER_BIN=${PWD}/${DEST_DIR}/piper"

#!/usr/bin/env bash
set -euo pipefail

CHANNEL="${1:-stable}"
shift || true

if [[ "$CHANNEL" != "stable" && "$CHANNEL" != "ea" ]]; then
  echo "Usage: $0 [stable|ea] [package1 package2 ...]" >&2
  exit 2
fi

if [[ "$#" -eq 0 ]]; then
  PACKAGES=(picoscope)
else
  PACKAGES=("$@")
fi

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
else
  echo "Cannot detect OS: /etc/os-release missing" >&2
  exit 1
fi

CODENAME="${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}"
if [[ -z "$CODENAME" ]]; then
  echo "Could not detect distro codename (expected VERSION_CODENAME/UBUNTU_CODENAME)." >&2
  exit 1
fi

if [[ "$CHANNEL" == "stable" ]]; then
  REPO_BASE="https://labs.picotech.com/picoscope7/debian"
else
  REPO_BASE="https://labs.picotech.com/rc/picoscope7/debian"
fi

KEY_URL="$REPO_BASE/picotech_public_key.asc"
KEYRING_PATH="/usr/share/keyrings/picotech-archive-keyring.gpg"
LIST_PATH="/etc/apt/sources.list.d/picoscope7.list"

echo "[INFO] Channel: $CHANNEL"
echo "[INFO] Repo: $REPO_BASE"
echo "[INFO] Codename: $CODENAME"
echo "[INFO] Packages: ${PACKAGES[*]}"

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required but not available." >&2
  exit 1
fi

if ! command -v wget >/dev/null 2>&1; then
  echo "[INFO] Installing wget and gpg prerequisites"
  sudo apt-get update
  sudo apt-get install -y wget gpg
fi

echo "[INFO] Installing PicoTech apt key"
wget -qO- "$KEY_URL" | gpg --dearmor | sudo tee "$KEYRING_PATH" >/dev/null

echo "[INFO] Writing apt source list"
echo "deb [signed-by=$KEYRING_PATH] $REPO_BASE $CODENAME main" | sudo tee "$LIST_PATH" >/dev/null

echo "[INFO] Updating apt cache"
sudo apt-get update

echo "[INFO] Installing packages"
sudo apt-get install -y "${PACKAGES[@]}"

echo "[DONE] Installation complete"
echo "[NEXT] You can inspect available Pico driver packages with:"
echo "       apt-cache search '^libps[0-9]'"

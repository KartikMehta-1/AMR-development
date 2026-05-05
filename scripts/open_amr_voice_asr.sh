#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${AMR_DEVPC_CONTAINER:-amr_devpc}"
DEVICE="${AMR_VOICE_DEVICE:-auto}"

if [[ $# -gt 0 && "$1" != --* ]]; then
  DEVICE="$1"
  shift
fi

if ! docker ps --filter "name=^/${CONTAINER_NAME}$" --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running. Launch navigation or start amr_devpc first." >&2
  exit 1
fi

voice_args=(--device "${DEVICE}" "$@")
printf -v voice_args_q ' %q' "${voice_args[@]}"

docker_tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then
  docker_tty_args=(-it)
fi

exec docker exec "${docker_tty_args[@]}" "${CONTAINER_NAME}" /entrypoint.sh bash -lc "
set -e
cd /workspaces/AMR-development/ros_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
exec ros2 run amr_voice voice_asr_node${voice_args_q}
"

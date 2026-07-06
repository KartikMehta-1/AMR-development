#!/usr/bin/env bash
set -euo pipefail

ORIN_HOST="${ORIN_HOST:-orin}"
CONTAINER="${AMR_ORIN_MOVEIT_CONTAINER:-amr_orin_moveit}"

remote_args=""
for arg in "$@"; do
  remote_args+=" $(printf '%q' "${arg}")"
done

ssh -t "${ORIN_HOST}" "docker exec -it ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros/driver_ws/install/setup.bash && source /workspaces/AMR-development/ros_ws/install/setup.bash && python3 /workspaces/AMR-development/scripts/so101_named_pose.py${remote_args}'"

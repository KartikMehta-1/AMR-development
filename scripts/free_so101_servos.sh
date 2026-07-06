#!/usr/bin/env bash
set -euo pipefail

ORIN_HOST="${ORIN_HOST:-orin}"
CONTAINER="${AMR_ORIN_MOVEIT_CONTAINER:-amr_orin_moveit}"

ssh "${ORIN_HOST}" "docker exec ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash && source /opt/ros/driver_ws/install/setup.bash && source /workspaces/AMR-development/ros_ws/install/setup.bash && ros2 service call /so101/free_servos std_srvs/srv/Trigger {}'"

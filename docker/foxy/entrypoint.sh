#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "/opt/ros/driver_ws/install/setup.bash" ]; then
  source "/opt/ros/driver_ws/install/setup.bash"
fi
if [ -f "/workspaces/ros_ws/install/setup.bash" ]; then
  source "/workspaces/ros_ws/install/setup.bash"
fi

exec "$@"

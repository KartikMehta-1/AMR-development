#!/usr/bin/env bash
set -e

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.sh"
elif [ -f "/opt/ros/${ROS_DISTRO}/install/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/install/setup.bash"
elif [ -f "/opt/ros/${ROS_DISTRO}/install/setup.sh" ]; then
  source "/opt/ros/${ROS_DISTRO}/install/setup.sh"
else
  echo "ROS setup not found under /opt/ros/${ROS_DISTRO}" >&2
  ls -la "/opt/ros/${ROS_DISTRO}" || true
  ls -la "/opt/ros/${ROS_DISTRO}/install" || true
  exit 1
fi
if [ -f "/opt/ros/driver_ws/install/setup.bash" ]; then
  source "/opt/ros/driver_ws/install/setup.bash"
fi
if [ -f "/workspaces/ros_ws/install/setup.bash" ]; then
  source "/workspaces/ros_ws/install/setup.bash"
fi

exec "$@"

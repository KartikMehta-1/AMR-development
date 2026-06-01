#!/usr/bin/env bash
set -e

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "ROS setup not found under /opt/ros/${ROS_DISTRO}" >&2
  exit 1
fi

if [ "${AMR_SOURCE_WORKSPACE:-0}" = "1" ] && [ -f "/workspaces/AMR-development/ros_ws/install/setup.bash" ]; then
  source "/workspaces/AMR-development/ros_ws/install/setup.bash"
fi

if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi

if [ -z "${ROS_DOMAIN_ID:-}" ]; then
  export ROS_DOMAIN_ID=0
fi

if [ -z "${ROS_LOCALHOST_ONLY:-}" ]; then
  export ROS_LOCALHOST_ONLY=0
fi

iface=""
ip_addr=""
if command -v ip >/dev/null 2>&1; then
  iface="$(ip -4 -o addr show 2>/dev/null | awk '{print $2}' | sort -u | \
    grep -E -v '^(lo|docker0|br-.*|veth.*)$' | head -n1 || true)"
  if [ -n "${iface}" ]; then
    ip_addr="$(ip -4 -o addr show "${iface}" 2>/dev/null | awk '{split($4,a,"/"); print a[1]; exit}')"
  fi
fi

if [ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ] && [ -z "${CYCLONEDDS_URI:-}" ] && [ -n "${ip_addr}" ]; then
  profile_file="/tmp/cyclonedds_${iface}.xml"
  peer_xml=""
  if [ -n "${AMR_CYCLONEDDS_PEER:-}" ]; then
    peer_xml="        <Peer Address=\"${AMR_CYCLONEDDS_PEER}\" />"
  fi
  cat > "${profile_file}" <<EOF
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="${ip_addr}" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
${peer_xml}
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
  export CYCLONEDDS_URI="file://${profile_file}"
fi

exec "$@"

#!/usr/bin/env bash
set -e

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.sh"
else
  echo "ROS setup not found under /opt/ros/${ROS_DISTRO}" >&2
  exit 1
fi

if [ -f "/opt/ros/driver_ws/install/setup.bash" ]; then
  source "/opt/ros/driver_ws/install/setup.bash"
fi

if [ -f "/workspaces/AMR-development/ros_ws/install/setup.bash" ]; then
  source "/workspaces/AMR-development/ros_ws/install/setup.bash"
elif [ -f "/workspaces/ros_ws/install/setup.bash" ]; then
  source "/workspaces/ros_ws/install/setup.bash"
fi

iface=""
ip_addr=""
if command -v ip >/dev/null 2>&1; then
  iface="$(ip -4 -o addr show 2>/dev/null | awk '{print $2}' | sort -u | \
    grep -E -v '^(lo|docker0|br-.*|veth.*|l4tbr0|rndis0|usb0|usb1)$' | head -n1 || true)"
  if [ -n "${iface}" ]; then
    ip_addr="$(ip -4 -o addr show "${iface}" 2>/dev/null | awk '{split($4,a,"/"); print a[1]; exit}')"
  fi
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

if [ "${RMW_IMPLEMENTATION:-}" = "rmw_fastrtps_cpp" ] && [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && [ -n "${ip_addr}" ]; then
  profile_file="/tmp/fastdds_${iface}.xml"
  cat > "${profile_file}" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>${ip_addr}</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="ros2_default" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
  export FASTRTPS_DEFAULT_PROFILES_FILE="${profile_file}"
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

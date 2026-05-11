ARG BASE_IMAGE=ros:foxy-ros-base
# Jetson Nano (JetPack 4.6.5 / L4T r32.7.6): dustynv doesn't publish r32.7.6 tags.
# Use the latest available JetPack 4 tag instead:
# --build-arg BASE_IMAGE=dustynv/ros:foxy-ros-base-l4t-r32.7.1
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=foxy
ARG TARGETARCH
ARG REALSENSE_ROS_BRANCH=r/4.56.4
ARG LIBREALSENSE_TAG=v2.56.4
ARG YDLIDAR_ROS_BRANCH=master
ARG YDLIDAR_SDK_BRANCH=master
ARG MICROROS_AGENT_BRANCH=foxy
ARG MICROROS_MSGS_BRANCH=foxy
ARG IMAGE_COMMON_BRANCH=foxy
ARG VISION_OPENCV_BRANCH=foxy
ARG XACRO_BRANCH=2.0.7
ARG CONTROL_MSGS_BRANCH=foxy-devel
ARG DIAGNOSTICS_BRANCH=ros2
ARG PYTHON_CMAKE_MODULE_BRANCH=foxy
ARG ANGLES_BRANCH=foxy-devel
ARG ACKERMANN_MSGS_BRANCH=ros2
ARG ROS2_CONTROL_BRANCH=foxy
ARG ROS2_CONTROLLERS_BRANCH=foxy
ARG REALTIME_TOOLS_BRANCH=foxy-devel
ARG CONTROL_TOOLBOX_BRANCH=foxy
ARG INSTALL_ROS_DEBS=auto
ARG BUILD_REALSENSE=auto

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN set -eux; \
    rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      gnupg; \
    OS_CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"; \
    HAVE_ROS_SNAPSHOT=0; \
    if grep -R "snapshots.ros.org/${ROS_DISTRO}/final" /etc/apt/sources.list /etc/apt/sources.list.d >/dev/null 2>&1; then \
      HAVE_ROS_SNAPSHOT=1; \
    fi; \
    if { [ "${INSTALL_ROS_DEBS}" = "1" ] || { [ "${INSTALL_ROS_DEBS}" = "auto" ] && [ "${OS_CODENAME}" != "bionic" ]; }; } && [ "${HAVE_ROS_SNAPSHOT}" != "1" ]; then \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --dearmor --yes --batch --no-tty -o /usr/share/keyrings/ros-archive-keyring.gpg; \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu ${OS_CODENAME} main" \
        > /etc/apt/sources.list.d/ros2.list; \
    fi; \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    OS_CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"; \
    ROS_DEBS=""; \
    OPENCV_DEB="libopencv-dev"; \
    BUILD_RS="${BUILD_REALSENSE}"; \
    if [ "${INSTALL_ROS_DEBS}" = "1" ] || { [ "${INSTALL_ROS_DEBS}" = "auto" ] && [ "${OS_CODENAME}" != "bionic" ]; }; then \
      ROS_DEBS="ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-xacro"; \
    fi; \
    if [ "${BUILD_RS}" = "auto" ]; then \
      if [ "${OS_CODENAME}" = "bionic" ] && [ "${TARGETARCH}" = "arm64" ]; then BUILD_RS=1; \
      elif [ "${OS_CODENAME}" = "bionic" ]; then BUILD_RS=0; \
      else BUILD_RS=1; fi; \
    fi; \
    EXTRA_REALSENSE_DEBS=""; \
    if [ "${BUILD_RS}" = "1" ] && [ "${OS_CODENAME}" = "bionic" ]; then \
      OPENCV_DEB=""; \
      EXTRA_REALSENSE_DEBS="libboost-dev libboost-python-dev python3-numpy python3-opencv"; \
    elif [ "${OS_CODENAME}" = "bionic" ]; then \
      OPENCV_DEB=""; \
    fi; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      curl \
      wget \
      alsa-utils \
      libportaudio2 \
      pkg-config \
      iproute2 \
      python3-colcon-common-extensions \
      python3-pip \
      python3-rosdep \
      python3-vcstool \
      python3-argcomplete \
      libssl-dev \
      libcurl4-openssl-dev \
      libusb-1.0-0-dev \
      libudev-dev \
      libgtk-3-dev \
      libglfw3-dev \
      libgl1-mesa-dev \
      libglu1-mesa-dev \
      libxinerama-dev \
      libxcursor-dev \
      libxi-dev \
      libcap-dev \
      libeigen3-dev \
      libfmt-dev \
      ${OPENCV_DEB} \
      ${EXTRA_REALSENSE_DEBS} \
      udev \
      ${ROS_DEBS} \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir sounddevice openwakeword

RUN rosdep init || true

RUN set -eux; \
    OS_CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"; \
    BUILD_RS="${BUILD_REALSENSE}"; \
    if [ "${BUILD_RS}" = "auto" ]; then \
      if [ "${OS_CODENAME}" = "bionic" ] && [ "${TARGETARCH}" = "arm64" ]; then BUILD_RS=1; \
      elif [ "${OS_CODENAME}" = "bionic" ]; then BUILD_RS=0; \
      else BUILD_RS=1; fi; \
    fi; \
    if [ "${BUILD_RS}" = "1" ]; then \
      git clone --branch ${LIBREALSENSE_TAG} --depth 1 \
        https://github.com/IntelRealSense/librealsense.git /tmp/librealsense; \
      mkdir -p /tmp/librealsense/build; \
      cd /tmp/librealsense/build; \
      if [ "${TARGETARCH}" = "arm64" ]; then \
        cmake .. -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_PYTHON_BINDINGS=OFF; \
      else \
        cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_PYTHON_BINDINGS=OFF; \
      fi; \
      make -j"$(nproc)"; \
      make install; \
      ldconfig; \
      rm -rf /tmp/librealsense; \
    fi

RUN git clone --branch ${YDLIDAR_SDK_BRANCH} --depth 1 \
    https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/ydlidar_sdk \
    && mkdir -p /tmp/ydlidar_sdk/build \
    && cd /tmp/ydlidar_sdk/build \
    && cmake .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/ydlidar_sdk

RUN mkdir -p /opt/ros/driver_ws/src
WORKDIR /opt/ros/driver_ws/src

RUN set -eux; \
    OS_CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"; \
    BUILD_RS="${BUILD_REALSENSE}"; \
    if [ "${BUILD_RS}" = "auto" ]; then \
      if [ "${OS_CODENAME}" = "bionic" ] && [ "${TARGETARCH}" = "arm64" ]; then BUILD_RS=1; \
      elif [ "${OS_CODENAME}" = "bionic" ]; then BUILD_RS=0; \
      else BUILD_RS=1; fi; \
    fi; \
    if [ "${OS_CODENAME}" = "bionic" ]; then \
      git clone --branch ${XACRO_BRANCH} --depth 1 \
        https://github.com/ros/xacro.git; \
    fi; \
    if [ "${BUILD_RS}" = "1" ] && [ "${OS_CODENAME}" = "bionic" ]; then \
      git clone --branch ${IMAGE_COMMON_BRANCH} --depth 1 \
        https://github.com/ros-perception/image_common.git; \
      git clone --branch ${VISION_OPENCV_BRANCH} --depth 1 \
        https://github.com/ros-perception/vision_opencv.git; \
      git clone --branch ${CONTROL_MSGS_BRANCH} --depth 1 \
        https://github.com/ros-controls/control_msgs.git; \
      git clone --branch ${ROS2_CONTROL_BRANCH} --depth 1 \
        https://github.com/ros-controls/ros2_control.git; \
      git clone --branch ${ROS2_CONTROLLERS_BRANCH} --depth 1 \
        https://github.com/ros-controls/ros2_controllers.git; \
      git clone --branch ${CONTROL_TOOLBOX_BRANCH} --depth 1 \
        https://github.com/ros-controls/control_toolbox.git; \
      git clone --depth 1 \
        https://github.com/ros-controls/ros2_control_cmake.git; \
      git clone --branch ${REALTIME_TOOLS_BRANCH} --depth 1 \
        https://github.com/ros-controls/realtime_tools.git; \
      git clone --branch ${DIAGNOSTICS_BRANCH} --depth 1 \
        https://github.com/ros/diagnostics.git; \
      git clone --branch ${ANGLES_BRANCH} --depth 1 \
        https://github.com/ros/angles.git; \
      git clone --branch ${ACKERMANN_MSGS_BRANCH} --depth 1 \
        https://github.com/ros-drivers/ackermann_msgs.git; \
      git clone --branch ${PYTHON_CMAKE_MODULE_BRANCH} --depth 1 \
        https://github.com/ros2/python_cmake_module.git; \
      if [ -d diagnostics/diagnostic_remote_logging ]; then \
        touch diagnostics/diagnostic_remote_logging/COLCON_IGNORE; \
      fi; \
    fi; \
    if [ "${BUILD_RS}" = "1" ]; then \
      git clone --branch ${REALSENSE_ROS_BRANCH} --depth 1 \
        https://github.com/IntelRealSense/realsense-ros.git; \
    fi
RUN set -eux; \
    hdr_dir="/opt/ros/driver_ws/src/realtime_tools/realtime_tools/include/realtime_tools"; \
    if [ ! -d "${hdr_dir}" ] && [ -d "/opt/ros/driver_ws/src/realtime_tools/include/realtime_tools" ]; then \
      hdr_dir="/opt/ros/driver_ws/src/realtime_tools/include/realtime_tools"; \
    fi; \
    if [ -d "${hdr_dir}" ] && [ -f "${hdr_dir}/realtime_publisher.hpp" ] && [ ! -f "${hdr_dir}/realtime_publisher.h" ]; then \
      ln -s realtime_publisher.hpp "${hdr_dir}/realtime_publisher.h"; \
    fi; \
    if [ -d "${hdr_dir}" ] && [ -f "${hdr_dir}/realtime_buffer.hpp" ] && [ ! -f "${hdr_dir}/realtime_buffer.h" ]; then \
      ln -s realtime_buffer.hpp "${hdr_dir}/realtime_buffer.h"; \
    fi; \
    if [ -d "${hdr_dir}" ] && [ -f "${hdr_dir}/realtime_box.hpp" ] && [ ! -f "${hdr_dir}/realtime_box.h" ]; then \
      ln -s realtime_box.hpp "${hdr_dir}/realtime_box.h"; \
    fi; \
    if [ -d "${hdr_dir}" ] && [ ! -f "${hdr_dir}/realtime_box.h" ] && [ -f "${hdr_dir}/realtime_box.hpp" ]; then \
      printf '%s\n' \
        '#pragma once' \
        '#if __has_include("realtime_tools/realtime_box.hpp")' \
        '#include "realtime_tools/realtime_box.hpp"' \
        '#endif' \
        > "${hdr_dir}/realtime_box.h"; \
    fi
RUN set -eux; \
    for hdr in /opt/ros/driver_ws/src/realtime_tools/**/include/realtime_tools/realtime_box.hpp; do \
      [ -f "$hdr" ] || continue; \
      dir="$(dirname "$hdr")"; \
      if [ ! -f "${dir}/realtime_box.h" ]; then \
        printf '%s\n' \
          '#pragma once' \
          '#if __has_include("realtime_tools/realtime_box.hpp")' \
          '#include "realtime_tools/realtime_box.hpp"' \
          '#endif' \
          > "${dir}/realtime_box.h"; \
      fi; \
    done
RUN git clone --branch ${YDLIDAR_ROS_BRANCH} --depth 1 \
    https://github.com/YDLIDAR/ydlidar_ros2_driver.git
RUN git clone --branch ${MICROROS_AGENT_BRANCH} --depth 1 \
    https://github.com/micro-ROS/micro-ROS-Agent.git
RUN git clone --branch ${MICROROS_MSGS_BRANCH} --depth 1 \
    https://github.com/micro-ROS/micro_ros_msgs.git
RUN python3 - <<'PY'
from pathlib import Path

os_release = Path("/etc/os-release").read_text()
if "VERSION_CODENAME=bionic" in os_release:
    path = Path("/opt/ros/driver_ws/src/micro-ROS-Agent/micro_ros_agent/cmake/SuperBuild.cmake")
    text = path.read_text()
    needle = "-DUAGENT_USE_SYSTEM_LOGGER:BOOL=${UAGENT_USE_SYSTEM_LOGGER}"
    insert = "\\n                -DUAGENT_LOGGER_PROFILE:BOOL=OFF"
    if needle in text and "UAGENT_LOGGER_PROFILE:BOOL=OFF" not in text:
        text = text.replace(needle, needle + insert)
        path.write_text(text)
PY

WORKDIR /opt/ros/driver_ws
RUN /bin/bash -lc 'set -ex; \
    setup=""; \
    for s in \
      /opt/ros/${ROS_DISTRO}/setup.sh \
      /opt/ros/${ROS_DISTRO}/setup.bash \
      /opt/ros/${ROS_DISTRO}/install/setup.sh \
      /opt/ros/${ROS_DISTRO}/install/setup.bash; do \
      if [ -f "$s" ]; then setup="$s"; break; fi; \
    done; \
    if [ -z "$setup" ]; then \
      echo "ROS setup not found under /opt/ros/${ROS_DISTRO}" >&2; \
      ls -la /opt/ros/${ROS_DISTRO} || true; \
      ls -la /opt/ros/${ROS_DISTRO}/install || true; \
      exit 1; \
    fi; \
    source "$setup"; \
    colcon build --merge-install --packages-select \
      ros2_control_cmake \
      realtime_tools \
      control_msgs \
      controller_manager_msgs \
      ros2_control_test_assets \
      hardware_interface \
      controller_interface \
      transmission_interface \
      controller_manager \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DCMAKE_EXE_LINKER_FLAGS=-lfmt \
      -DCMAKE_SHARED_LINKER_FLAGS=-lfmt'

RUN python3 - <<'PY'
from pathlib import Path
import shutil

install_inc = Path("/opt/ros/driver_ws/install/include/realtime_tools")
foxy_inc = Path("/opt/ros/foxy/install/include/realtime_tools")
src_candidates = [
    Path("/opt/ros/driver_ws/src/realtime_tools/realtime_tools/include/realtime_tools"),
    Path("/opt/ros/driver_ws/src/realtime_tools/include/realtime_tools"),
]

src_dir = next((p for p in src_candidates if p.is_dir()), None)
compat = """#pragma once
#include <mutex>
#include <utility>

namespace realtime_tools {
template <class T>
class RealtimeBox {
 public:
  RealtimeBox() = default;
  explicit RealtimeBox(const T & value) : value_(value) {}
  explicit RealtimeBox(T && value) : value_(std::move(value)) {}

  bool set(const T & value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
    return true;
  }
  bool set(T && value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(value);
    return true;
  }
  T get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }
  bool try_get(T & out) const {
    if (!mutex_.try_lock()) {
      return false;
    }
    out = value_;
    mutex_.unlock();
    return true;
  }

  void writeFromNonRT(const T & value) { set(value); }
  void writeFromNonRT(T && value) { set(std::move(value)); }
  const T * readFromRT() const { return &value_; }
  T * readFromNonRT() { return &value_; }

 private:
  mutable std::mutex mutex_;
  T value_{};
};
}  // namespace realtime_tools
"""
for target in (install_inc, foxy_inc):
    if not src_dir:
        continue
    target.mkdir(parents=True, exist_ok=True)
    for p in src_dir.glob("*"):
        if p.is_file() and p.suffix in {".hpp", ".h"}:
            shutil.copy2(p, target / p.name)

for d in [p for p in [install_inc, foxy_inc, src_dir] if p and p.is_dir()]:
    box_hpp = d / "realtime_box.hpp"
    if not box_hpp.exists():
        box_hpp.write_text(compat)
    for name in ("realtime_box", "realtime_buffer", "realtime_publisher"):
        h = d / f"{name}.h"
        hpp = d / f"{name}.hpp"
        if not h.exists() and hpp.exists():
            h.write_text(
                "#pragma once\n"
                f"#if __has_include(\"realtime_tools/{name}.hpp\")\n"
                f"#include \"realtime_tools/{name}.hpp\"\n"
                "#endif\n"
            )
PY

RUN cat <<'EOS' >/tmp/build_ros2_control_controllers.sh
#!/bin/bash
set -ex
setup=""
for s in \
  /opt/ros/${ROS_DISTRO}/setup.sh \
  /opt/ros/${ROS_DISTRO}/setup.bash \
  /opt/ros/${ROS_DISTRO}/install/setup.sh \
  /opt/ros/${ROS_DISTRO}/install/setup.bash; do
  if [ -f "$s" ]; then setup="$s"; break; fi
done
if [ -z "$setup" ]; then
  echo "ROS setup not found under /opt/ros/${ROS_DISTRO}" >&2
  ls -la /opt/ros/${ROS_DISTRO} || true
  ls -la /opt/ros/${ROS_DISTRO}/install || true
  exit 1
fi
source "$setup"
if [ -f /opt/ros/driver_ws/install/setup.bash ]; then
  source /opt/ros/driver_ws/install/setup.bash
elif [ -f /opt/ros/driver_ws/install/setup.sh ]; then
  source /opt/ros/driver_ws/install/setup.sh
fi
src_dir=""
for cand in \
  /opt/ros/driver_ws/src/realtime_tools/realtime_tools/include/realtime_tools \
  /opt/ros/driver_ws/src/realtime_tools/include/realtime_tools; do
  if [ -d "$cand" ]; then
    src_dir="$cand"
    break
  fi
done
if [ -n "$src_dir" ]; then
  if [ ! -f "$src_dir/realtime_box.hpp" ]; then
    cat > "$src_dir/realtime_box.hpp" <<'EOF'
#pragma once
#include <mutex>
#include <utility>

namespace realtime_tools {
template <class T>
class RealtimeBox {
 public:
  RealtimeBox() = default;
  explicit RealtimeBox(const T & value) : value_(value) {}
  explicit RealtimeBox(T && value) : value_(std::move(value)) {}

  bool set(const T & value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = value;
    return true;
  }
  bool set(T && value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(value);
    return true;
  }
  T get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }
  bool try_get(T & out) const {
    if (!mutex_.try_lock()) {
      return false;
    }
    out = value_;
    mutex_.unlock();
    return true;
  }

  void writeFromNonRT(const T & value) { set(value); }
  void writeFromNonRT(T && value) { set(std::move(value)); }
  const T * readFromRT() const { return &value_; }
  T * readFromNonRT() { return &value_; }

 private:
  mutable std::mutex mutex_;
  T value_{};
};
}  // namespace realtime_tools
EOF
  fi
  for target in /opt/ros/driver_ws/install/include/realtime_tools /opt/ros/foxy/install/include/realtime_tools; do
    mkdir -p "$target"
    cp -a "$src_dir/"*.hpp "$target/" 2>/dev/null || true
    cp -a "$src_dir/"*.h "$target/" 2>/dev/null || true
  done
fi
rt_dirs=(
  /opt/ros/driver_ws/install/include/realtime_tools
  /opt/ros/driver_ws/src/realtime_tools/include/realtime_tools
  /opt/ros/driver_ws/src/realtime_tools/realtime_tools/include/realtime_tools
  /opt/ros/foxy/install/include/realtime_tools
)
for dir in "${rt_dirs[@]}"; do
  if [ -d "$dir" ]; then
    for h in realtime_box realtime_buffer realtime_publisher; do
      if [ ! -f "$dir/${h}.h" ] && [ -f "$dir/${h}.hpp" ]; then
        cat >"$dir/${h}.h" <<EOF
#pragma once
#if __has_include("realtime_tools/${h}.hpp")
#include "realtime_tools/${h}.hpp"
#endif
EOF
      fi
    done
  fi
done
echo "realtime_tools headers (driver_ws install):"
ls -la /opt/ros/driver_ws/install/include/realtime_tools || true
echo "realtime_tools headers (foxy install):"
ls -la /opt/ros/foxy/install/include/realtime_tools || true
colcon build --merge-install --packages-select \
  diff_drive_controller \
  joint_state_broadcaster \
  forward_command_controller \
  force_torque_sensor_broadcaster \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_EXE_LINKER_FLAGS=-lfmt \
  -DCMAKE_SHARED_LINKER_FLAGS=-lfmt
colcon build --merge-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF \
  -DCMAKE_EXE_LINKER_FLAGS=-lfmt \
  -DCMAKE_SHARED_LINKER_FLAGS=-lfmt
rm -rf build log
EOS
RUN /bin/bash /tmp/build_ros2_control_controllers.sh && rm /tmp/build_ros2_control_controllers.sh

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN if ! getent group ${GID} >/dev/null; then groupadd -g ${GID} ${USERNAME}; fi \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && groupadd -f dialout \
    && usermod -a -G dialout ${USERNAME}

RUN sed -i '\#^source /opt/ros/.*/setup.bash$#d; \#^source /opt/ros/driver_ws/install/setup.bash$#d' /etc/bash.bashrc \
    && cat <<'EOS' >> /etc/bash.bashrc
# ros env
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.sh"
elif [ -f "/opt/ros/${ROS_DISTRO}/install/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/install/setup.bash"
elif [ -f "/opt/ros/${ROS_DISTRO}/install/setup.sh" ]; then
  source "/opt/ros/${ROS_DISTRO}/install/setup.sh"
fi
if [ -f "/opt/ros/driver_ws/install/setup.bash" ]; then
  source "/opt/ros/driver_ws/install/setup.bash"
fi
EOS

WORKDIR /workspaces/ros_ws

RUN mkdir -p /opt/ros/rviz
COPY docker/foxy/rviz/amr_lidar.rviz /opt/ros/rviz/amr_lidar.rviz

COPY docker/foxy/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

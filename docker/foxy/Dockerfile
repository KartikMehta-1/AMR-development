ARG BASE_IMAGE=ros:foxy-ros-base
# Jetson Nano (JetPack 4.6.5 / L4T r32.7.6): use
# --build-arg BASE_IMAGE=dustynv/ros:foxy-ros-base-l4t-r32.7.6
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=foxy
ARG TARGETARCH
ARG REALSENSE_ROS_BRANCH=r/4.56.4
ARG LIBREALSENSE_TAG=v2.56.4
ARG YDLIDAR_ROS_BRANCH=master
ARG YDLIDAR_SDK_BRANCH=master
ARG MICROROS_AGENT_BRANCH=foxy
ARG MICROROS_MSGS_BRANCH=foxy
ARG INSTALL_ROS_DEBS=auto
ARG BUILD_REALSENSE=auto

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN rm -f /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2-latest.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor --yes --batch --no-tty -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu bionic main" \
      > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    OS_CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"; \
    ROS_DEBS=""; \
    OPENCV_DEB="libopencv-dev"; \
    if [ "${INSTALL_ROS_DEBS}" = "1" ] || { [ "${INSTALL_ROS_DEBS}" = "auto" ] && [ "${OS_CODENAME}" != "bionic" ]; }; then \
      ROS_DEBS="ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-xacro"; \
    fi; \
    if [ "${OS_CODENAME}" = "bionic" ]; then \
      OPENCV_DEB=""; \
    fi; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      curl \
      wget \
      pkg-config \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      python3-argcomplete \
      libssl-dev \
      libusb-1.0-0-dev \
      libudev-dev \
      libgtk-3-dev \
      libglfw3-dev \
      libgl1-mesa-dev \
      libglu1-mesa-dev \
      libxinerama-dev \
      libxcursor-dev \
      libxi-dev \
      libfmt-dev \
      ${OPENCV_DEB} \
      udev \
      ${ROS_DEBS} \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

RUN set -eux; \
    OS_CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"; \
    BUILD_RS="${BUILD_REALSENSE}"; \
    if [ "${BUILD_RS}" = "auto" ]; then \
      if [ "${OS_CODENAME}" = "bionic" ]; then BUILD_RS=0; else BUILD_RS=1; fi; \
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
      if [ "${OS_CODENAME}" = "bionic" ]; then BUILD_RS=0; else BUILD_RS=1; fi; \
    fi; \
    if [ "${BUILD_RS}" = "1" ]; then \
      git clone --branch ${REALSENSE_ROS_BRANCH} --depth 1 \
        https://github.com/IntelRealSense/realsense-ros.git; \
    fi
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
    colcon build --merge-install --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXE_LINKER_FLAGS=-lfmt \
      -DCMAKE_SHARED_LINKER_FLAGS=-lfmt; \
    rm -rf build log'

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

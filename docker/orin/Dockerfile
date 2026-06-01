ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=humble

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-c"]

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      gnupg \
      lsb-release \
      software-properties-common; \
    add-apt-repository universe; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor --yes --batch --no-tty -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${VERSION_CODENAME}") main" \
      > /etc/apt/sources.list.d/ros2.list; \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      curl \
      wget \
      pkg-config \
      iproute2 \
      iputils-ping \
      less \
      nano \
      tmux \
      udev \
      usbutils \
      v4l-utils \
      python3-pip \
      python3-requests \
      python3-tqdm \
      python3-argcomplete \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
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
      libopencv-dev \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-tf-transformations; \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

ARG LIBREALSENSE_TAG=v2.57.7
RUN set -eux; \
    git clone --branch ${LIBREALSENSE_TAG} --depth 1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense; \
    cmake -S /tmp/librealsense -B /tmp/librealsense/build \
      -DFORCE_RSUSB_BACKEND=ON \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_GRAPHICAL_EXAMPLES=OFF \
      -DBUILD_PYTHON_BINDINGS=OFF; \
    cmake --build /tmp/librealsense/build -j"$(nproc)"; \
    cmake --install /tmp/librealsense/build; \
    ldconfig; \
    rm -rf /tmp/librealsense

ARG YDLIDAR_SDK_BRANCH=master
RUN set -eux; \
    git clone --branch ${YDLIDAR_SDK_BRANCH} --depth 1 https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/ydlidar_sdk; \
    cmake -S /tmp/ydlidar_sdk -B /tmp/ydlidar_sdk/build; \
    cmake --build /tmp/ydlidar_sdk/build -j"$(nproc)"; \
    cmake --install /tmp/ydlidar_sdk/build; \
    ldconfig; \
    rm -rf /tmp/ydlidar_sdk

RUN mkdir -p /opt/ros/driver_ws/src
WORKDIR /opt/ros/driver_ws/src

ARG REALSENSE_ROS_BRANCH=ros2-development
ARG YDLIDAR_ROS_BRANCH=master
ARG MICROROS_AGENT_BRANCH=humble
ARG MICROROS_MSGS_BRANCH=humble
RUN set -eux; \
    git clone --branch ${YDLIDAR_ROS_BRANCH} --depth 1 https://github.com/YDLIDAR/ydlidar_ros2_driver.git; \
    git clone --branch ${MICROROS_AGENT_BRANCH} --depth 1 https://github.com/micro-ROS/micro-ROS-Agent.git; \
    git clone --branch ${MICROROS_MSGS_BRANCH} --depth 1 https://github.com/micro-ROS/micro_ros_msgs.git; \
    git clone --branch ${REALSENSE_ROS_BRANCH} --depth 1 https://github.com/IntelRealSense/realsense-ros.git; \
    sed -i \
      -e 's/node->declare_parameter("port");/node->declare_parameter("port", str_optvalue);/' \
      -e 's/node->declare_parameter("ignore_array");/node->declare_parameter("ignore_array", str_optvalue);/' \
      -e 's/node->declare_parameter("frame_id");/node->declare_parameter("frame_id", frame_id);/' \
      -e 's/node->declare_parameter("baudrate");/node->declare_parameter("baudrate", optval);/' \
      -e 's/node->declare_parameter("lidar_type");/node->declare_parameter("lidar_type", optval);/' \
      -e 's/node->declare_parameter("device_type");/node->declare_parameter("device_type", optval);/' \
      -e 's/node->declare_parameter("sample_rate");/node->declare_parameter("sample_rate", optval);/' \
      -e 's/node->declare_parameter("abnormal_check_count");/node->declare_parameter("abnormal_check_count", optval);/' \
      -e 's/node->declare_parameter("intensity_bit");/node->declare_parameter("intensity_bit", optval);/' \
      -e 's/node->declare_parameter("fixed_resolution");/node->declare_parameter("fixed_resolution", b_optvalue);/' \
      -e 's/node->declare_parameter("reversion");/node->declare_parameter("reversion", b_optvalue);/' \
      -e 's/node->declare_parameter("inverted");/node->declare_parameter("inverted", b_optvalue);/' \
      -e 's/node->declare_parameter("auto_reconnect");/node->declare_parameter("auto_reconnect", b_optvalue);/' \
      -e 's/node->declare_parameter("isSingleChannel");/node->declare_parameter("isSingleChannel", b_optvalue);/' \
      -e 's/node->declare_parameter("intensity");/node->declare_parameter("intensity", b_optvalue);/' \
      -e 's/node->declare_parameter("support_motor_dtr");/node->declare_parameter("support_motor_dtr", b_optvalue);/' \
      -e 's/node->declare_parameter("debug");/node->declare_parameter("debug", b_optvalue);/' \
      -e 's/node->declare_parameter("angle_max");/node->declare_parameter("angle_max", f_optvalue);/' \
      -e 's/node->declare_parameter("angle_min");/node->declare_parameter("angle_min", f_optvalue);/' \
      -e 's/node->declare_parameter("range_max");/node->declare_parameter("range_max", f_optvalue);/' \
      -e 's/node->declare_parameter("range_min");/node->declare_parameter("range_min", f_optvalue);/' \
      -e 's/node->declare_parameter("frequency");/node->declare_parameter("frequency", f_optvalue);/' \
      -e 's/node->declare_parameter("invalid_range_is_inf");/node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);/' \
      -e 's/node->declare_parameter("m1_mode");/node->declare_parameter("m1_mode", i_v);/' \
      -e 's/node->declare_parameter("m2_mode");/node->declare_parameter("m2_mode", i_v);/' \
      -e 's/node->declare_parameter("m3_mode");/node->declare_parameter("m3_mode", i_v);/' \
      ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp; \
    sed -i \
      -e 's/node_executable=/executable=/g' \
      -e 's/node_name=/name=/g' \
      -e 's/node_namespace=/namespace=/g' \
      ydlidar_ros2_driver/launch/ydlidar_launch.py \
      ydlidar_ros2_driver/launch/ydlidar_launch_view.py

WORKDIR /opt/ros/driver_ws
RUN set -ex; \
    source "/opt/ros/${ROS_DISTRO}/setup.bash"; \
    rosdep update; \
    apt-get update; \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}" \
      --skip-keys "librealsense2 launch_pytest"; \
    rm -rf /var/lib/apt/lists/*; \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF; \
    rm -rf build log

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-cyclonedds-cpp; \
    rm -rf /var/lib/apt/lists/*

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN set -eux; \
    if ! getent group ${GID} >/dev/null; then groupadd -g ${GID} ${USERNAME}; fi; \
    useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME}; \
    groupadd -f dialout; \
    groupadd -f video; \
    groupadd -f render; \
    groupadd -f plugdev; \
    usermod -a -G dialout,video,render,plugdev ${USERNAME}

RUN printf '%s\n' \
    '' \
    '# AMR Orin ROS environment' \
    'if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then' \
    '  source "/opt/ros/${ROS_DISTRO}/setup.bash"' \
    'fi' \
    'if [ -f "/opt/ros/driver_ws/install/setup.bash" ]; then' \
    '  source "/opt/ros/driver_ws/install/setup.bash"' \
    'fi' \
    'if [ -f "/workspaces/AMR-development/ros_ws/install/setup.bash" ]; then' \
    '  source "/workspaces/AMR-development/ros_ws/install/setup.bash"' \
    'fi' \
    >> /etc/bash.bashrc

WORKDIR /workspaces/AMR-development/ros_ws

COPY docker/orin/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

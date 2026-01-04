ARG BASE_IMAGE=ros:foxy-ros-base
FROM ${BASE_IMAGE}

ARG ROS_DISTRO=foxy
ARG TARGETARCH
ARG REALSENSE_ROS_BRANCH=r/4.56.4
ARG LIBREALSENSE_TAG=v2.56.4
ARG YDLIDAR_ROS_BRANCH=master
ARG YDLIDAR_SDK_BRANCH=master
ARG MICROROS_AGENT_BRANCH=foxy
ARG MICROROS_MSGS_BRANCH=foxy

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
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
    libopencv-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-xacro \
    udev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

RUN git clone --branch ${LIBREALSENSE_TAG} --depth 1 \
    https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && mkdir -p /tmp/librealsense/build \
    && cd /tmp/librealsense/build \
    && if [ "${TARGETARCH}" = "arm64" ]; then \
         cmake .. -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_PYTHON_BINDINGS=OFF; \
       else \
         cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_PYTHON_BINDINGS=OFF; \
       fi \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/librealsense

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

RUN git clone --branch ${REALSENSE_ROS_BRANCH} --depth 1 \
    https://github.com/IntelRealSense/realsense-ros.git
RUN git clone --branch ${YDLIDAR_ROS_BRANCH} --depth 1 \
    https://github.com/YDLIDAR/ydlidar_ros2_driver.git
RUN git clone --branch ${MICROROS_AGENT_BRANCH} --depth 1 \
    https://github.com/micro-ROS/micro-ROS-Agent.git
RUN git clone --branch ${MICROROS_MSGS_BRANCH} --depth 1 \
    https://github.com/micro-ROS/micro_ros_msgs.git

WORKDIR /opt/ros/driver_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN if ! getent group ${GID} >/dev/null; then groupadd -g ${GID} ${USERNAME}; fi \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && groupadd -f dialout \
    && usermod -a -G dialout ${USERNAME}

RUN printf '\nsource /opt/ros/%s/setup.bash\nsource /opt/ros/driver_ws/install/setup.bash\n' "${ROS_DISTRO}" >> /etc/bash.bashrc

WORKDIR /workspaces/ros_ws

RUN mkdir -p /opt/ros/rviz
COPY docker/foxy/rviz/amr_lidar.rviz /opt/ros/rviz/amr_lidar.rviz

COPY docker/foxy/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      bash-completion \
      iproute2 \
      mesa-utils \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-humble-cv-bridge \
      ros-humble-image-transport \
      ros-humble-image-transport-plugins \
      ros-humble-joint-state-publisher \
      ros-humble-joint-state-publisher-gui \
      ros-humble-nav2-rviz-plugins \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rqt-image-view \
      ros-humble-rviz2 \
      ros-humble-tf2-tools \
      ros-humble-xacro; \
    rm -rf /var/lib/apt/lists/*

COPY docker/nuc-humble-rviz/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspaces/AMR-development
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

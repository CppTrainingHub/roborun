FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
      build-essential \
      cmake \
      git \
      libboost-regex-dev \
      libzmq3-dev \
      python3-colcon-common-extensions \
      python3-pip \
      python3-pytest \
      ros-humble-ament-cmake-gtest \
      ros-humble-rclcpp-action \
      ros-humble-sensor-msgs \
    && python3 -m pip install --no-cache-dir 'cmake>=3.24,<4' \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

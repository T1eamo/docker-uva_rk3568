ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base AS builder
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /ros_app

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-rclcpp \
      ros-${ROS_DISTRO}-std-msgs && \
    rm -rf /var/lib/apt/lists/*

COPY ./src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM ros:${ROS_DISTRO}-ros-base AS runtime
ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]
WORKDIR /ros_app

COPY --from=builder /ros_app/install /ros_app/install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros_app/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros_app/install/setup.bash && ros2 run test_pkg test_node"]

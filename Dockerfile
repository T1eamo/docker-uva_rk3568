ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-base AS builder
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /ros_app

RUN test "$(dpkg --print-architecture)" = "arm64"

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      git \
      pkg-config \
      libdrm-dev \
      libopencv-dev \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-ament-index-cpp \
      ros-${ROS_DISTRO}-rclcpp \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-std-msgs && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch main https://github.com/airockchip/librga.git /tmp/librga && \
    cp -r /tmp/librga/include/. /usr/local/include/ && \
    cp /tmp/librga/libs/Linux/gcc-aarch64/librga.so /usr/local/lib/librga.so && \
    ldconfig

COPY ./src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select uva_pkg --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM ros:${ROS_DISTRO}-ros-base AS runtime
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV LD_LIBRARY_PATH=/ros_app/install/lib
SHELL ["/bin/bash", "-c"]
WORKDIR /ros_app

RUN test "$(dpkg --print-architecture)" = "arm64"

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      libdrm2 \
      libgl1 \
      libglib2.0-0 \
      libgtk-3-0 \
      libopencv-dev \
      ros-${ROS_DISTRO}-ament-index-cpp && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/lib/librga.so /usr/local/lib/librga.so
RUN ldconfig

COPY --from=builder /ros_app/install /ros_app/install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros_app/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros_app/install/setup.bash && ros2 run uva_pkg uva_node"]

ARG ROS_DISTRO="foxy"

FROM osrf/ros:$ROS_DISTRO-desktop

WORKDIR /workspace

RUN apt-get update \
    && apt-get install -y --no-install-recommends \ 
        libpng-dev libtiff5-dev libjpeg-dev \
        python3-opencv \
        python3-setuptools \
        python3-wheel \
        python3-pip \
        ros-$ROS_DISTRO-ackermann-msgs \
        ros-$ROS_DISTRO-derived-object-msgs \
        ros-$ROS_DISTRO-cv-bridge \
        ros-$ROS_DISTRO-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install transforms3d

ENV CARLA_ROOT "/workspace/CARLA"

ENV SCRIPTS_WS "/workspace/scripts"
ENV ROS2_WS "/workspace/ws"

COPY .tmp/scripts "${SCRIPTS_WS}"
COPY .tmp/ws "${ROS2_WS}"

COPY .tmp/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]

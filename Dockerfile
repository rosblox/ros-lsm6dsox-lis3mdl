ARG ROS_DISTRO

FROM ros:${ROS_DISTRO}-ros-core

RUN apt update && apt install -y --no-install-recommends python3-rpi.gpio python3-pip python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
RUN pip3 install adafruit-circuitpython-lsm6ds adafruit-circuitpython-lis3mdl


WORKDIR /colcon_ws/src

COPY ros_lsm6dsox_lis3mdl .

WORKDIR /colcon_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

WORKDIR /

COPY ros_entrypoint.sh .

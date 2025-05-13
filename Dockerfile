ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO}-ros-core

RUN apt update && apt install -y --no-install-recommends gcc python3-lgpio python3-dev python3-pip python3-colcon-common-extensions && rm -rf /var/lib/apt/lists/*
RUN pip3 install adafruit-extended-bus adafruit-circuitpython-lsm6ds adafruit-circuitpython-lis3mdl

COPY ros_entrypoint.sh /ros_entrypoint.sh

WORKDIR /colcon_ws

COPY ros_lsm6dsox_lis3mdl src/ros_lsm6dsox_lis3mdl

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --event-handlers console_direct+ --cmake-args ' -DCMAKE_BUILD_TYPE=Release'

ENV LAUNCH_COMMAND='ros2 run ros_lsm6dsox_lis3mdl ros_lsm6dsox_lis3mdl_publisher'

# Create build and run aliases
RUN echo 'alias build="colcon build --symlink-install  --event-handlers console_direct+"' >> /etc/bash.bashrc && \
echo 'alias run="su - ros --whitelist-environment=\"ROS_DOMAIN_ID\" /run.sh"' >> /etc/bash.bashrc && \
echo "source /colcon_ws/install/setup.bash; echo UID: $UID; echo ROS_DOMAIN_ID: $ROS_DOMAIN_ID; $LAUNCH_COMMAND" >> /run.sh && chmod +x /run.sh
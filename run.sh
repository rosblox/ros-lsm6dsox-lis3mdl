#!/bin/bash

REPOSITORY_NAME="$(basename "$(dirname -- "$( readlink -f -- "$0"; )")")"

docker run -it --rm \
--network=host \
--ipc=host --pid=host \
--privileged \
--env UID=$(id -u) \
--env GID=$(id -g) \
--volume ./ros_lsm6dsox_lis3mdl:/colcon_ws/src/ros_lsm6dsox_lis3mdl \
ghcr.io/rosblox/${REPOSITORY_NAME}:humble

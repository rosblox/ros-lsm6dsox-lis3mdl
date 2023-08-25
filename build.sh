#!/bin/bash

ROS_DISTRO=humble

REPOSITORY_NAME="$(basename "$(dirname -- "$( readlink -f -- "$0"; )")")"

docker build --progress=plain --build-arg ROS_DISTRO=${ROS_DISTRO} -t ghcr.io/rosblox/${REPOSITORY_NAME}:${ROS_DISTRO} .
#!/bin/bash

# Map host's display socket to docker
# DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
# DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
# DOCKER_ARGS+=("-v ./volume/:/root/DEVELOPMENT/volume:rw")
DOCKER_ARGS+=("-e DISPLAY=${DISPLAY}")

image_name="harmonic-robotcore"
# container_name="container1"

force_option=false

docker run -it --rm=false \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -e DISPLAY=$DISPLAY \
    -v $PWD/ros_ws:/framework_ws/src/acceleration/image_proc/ \
    -v $PWD/rosbag/:/framework_ws/src/acceleration/rosbag/ \
    $@ \
    "$image_name" \
    bash

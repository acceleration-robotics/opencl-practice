#!/bin/bash

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
# DOCKER_ARGS+=("-v ./volume/:/root/DEVELOPMENT/volume:rw")
DOCKER_ARGS+=("-e DISPLAY=${DISPLAY}")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
xhost +local:root

image_name="nvidia-foxy-opencl"
# container_name="container1"

force_option=false

docker run -it --rm=false \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -e DISPLAY=$DISPLAY \
    --gpus all \
    --runtime nvidia \
    -v $PWD:/home/user/DEVELOPMENT/ \
    -v /home/anushree/acceleration_robotics/opencl-practice/image_processing:/home/user/image_processing/ \
    $@ \
    "$image_name" \
    bash

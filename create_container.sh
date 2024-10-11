#!/bin/bash

# Allows local root to access the X server
xhost local:root

# Define the path for the X11 authority file
XAUTH=/tmp/.docker.xauth

# Use the current directory as the host workspace directory
HOST_WORKSPACE_DIR="$PWD/src"

# Run the Docker container with necessary environment variables and volume mounts for X11
docker run -it \
    --name=ros2_tb3_dev_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    ros2_tb3 \
    bash

echo "Container started."

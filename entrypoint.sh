#!/bin/bash
set -e

# Source the ROS 2 setup file to initialize the environment
source "/opt/ros/humble/setup.bash"

# Set the TurtleBot model as an environment variable
export TURTLEBOT3_MODEL=waffle

# Execute the passed command or bash if no command is passed
exec "$@"
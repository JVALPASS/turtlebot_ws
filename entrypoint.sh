#!/bin/bash
set -e

# Ensure the ROS environment is sourced on container startup
if ! grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

# Set the default TurtleBot3 model
if ! grep -Fxq "export TURTLEBOT3_MODEL=waffle" ~/.bashrc; then
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
fi

# Execute the passed command or fallback to bash
exec "$@"
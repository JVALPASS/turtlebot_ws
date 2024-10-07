# Use an official ROS 2 desktop image as a parent image
FROM osrf/ros:humble-desktop

# Install additional ROS packages and utilities
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    python3-colcon-common-extensions \
    git

# Set the working directory to /app
WORKDIR /app

# Copy the current directory contents into the container at /app/TURTLEBOT_WS
COPY . /app/TURTLEBOT_WS

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh

# Make entrypoint.sh executable
RUN chmod +x /entrypoint.sh

# Use entrypoint.sh to setup the environment when the container starts
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

# Command to run when starting the container
CMD ["bash"]

# Use an official ROS 2 desktop image as a parent image
FROM osrf/ros:humble-desktop

# Install additional ROS packages and utilities
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-slam-toolbox \
    ros-humble-nav2-simple-commander \
    ros-humble-tf-transformations -y \
    ros-humble-ros2-control -y \
    ros-humble-ros2-controllers -y \
    ros-humble-xacro \
    ros-humble-ros-gz* -y \
    ros-humble-*-ros2-control -y \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-localization -y \
    ros-humble-joy \
    ros-humble-joy-teleop -y \
    python3-transforms3d \
    python3-colcon-common-extensions \
    terminator \
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

# Default command to run when starting the container
CMD ["bash"]
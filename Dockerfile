#This is the image of ROS2 that is pulled ()
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    python3-colcon-common-extensions \
    git


# here we specify the work directory inside of our container, everything will be placed inside
WORKDIR /app

COPY . /app/TURTLEBOT_WS

# Copy the entrypoint script into the container
COPY entrypoint.sh /entrypoint.sh

# Use the entrypoint script to handle setup and execution
ENTRYPOINT [ "/bin/bash", "/entrypoint.sh"]

# Default command to run (if no other command is passed)
CMD ["bash"]
# TurtleBot3 Workspace

This repository contains the necessary commands to launch and operate the TurtleBot3 robot in a simulated environment using Gazebo, SLAM, and navigation in ROS 2. Below are the steps and commands for launching the robot, enabling navigation, SLAM, and saving/loading maps.

## Prerequisites
Ensure you have the following packages installed:
- `turtlebot3_gazebo`
- `nav2_bringup`
- `slam_toolbox`
- `rviz2`

You also need to source your ROS 2 environment before running any commands:
```bash
source /opt/ros/<your_ros_distro>/setup.bash
source <your_workspace>/install/setup.bash
```

Replace `<your_ros_distro>` with your ROS 2 distribution (e.g., `humble`, `foxy`), and `<your_workspace>` with your workspace path.

## Launch TurtleBot3 in Gazebo
To launch the TurtleBot3 model in Gazebo with its world environment:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

This command will bring up the TurtleBot3 robot in the default Gazebo simulation environment.

## Start the Navigation Stack
Once the simulation is running, start the navigation stack using the launch file from the `nav2_bringup` package:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
The `use_sim_time:=true` flag tells ROS 2 to use the simulation time from Gazebo for synchronization.

## Enable SLAM with SLAM Toolbox
To enable SLAM (Simultaneous Localization and Mapping) and start mapping the environment, use the following command:
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
This command launches the SLAM Toolbox in asynchronous mode, allowing the robot to build a map while moving through the environment.

## Launch RViz2
RViz2 is a 3D visualization tool that helps visualize the robot’s movement, sensor data, and map:
```bash
rviz2
```
Make sure to load the correct RViz configuration file if needed, or configure it to show the robot model, map, and sensor data.

## Save the Map
Once the mapping is complete, you can save the generated map using the `nav2_map_server` package:
```bash
ros2 run nav2_map_server map_saver_cli -f maps/myworld
```
This will save the map in the `maps/` directory as `myworld`. You can specify a different path or filename if needed.

## Load the Saved Map and Start Navigation
To use the saved map for navigation, relaunch the navigation stack with the map parameter:
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map:=maps/myworld.yaml
```
Make sure to provide the path to the saved `.yaml` map file.

## Additional Notes
- You can adjust simulation parameters (like the robot's world) by editing the respective launch files in the `turtlebot3_gazebo` package.
- If you're using a different TurtleBot3 model (e.g., Waffle Pi or Burger), ensure to export the model type using:
  ```bash
  export TURTLEBOT3_MODEL=burger
  ```
  Replace `burger` with `waffle_pi` or other models if needed.
```

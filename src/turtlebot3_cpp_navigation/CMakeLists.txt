cmake_minimum_required(VERSION 3.8)
project(turtlebot3_cpp_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable nav2_to_point_goal
add_executable(nav2_to_point_goal src/nav2_to_point_goal.cpp)

# Add executable for follow_waypoints
add_executable(follow_waypoints src/follow_waypoints.cpp)


# Link libraries nav2_to_point_goal
ament_target_dependencies(nav2_to_point_goal
    rclcpp
    rclcpp_action
    nav2_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
)

# Link libraries for follow_waypoints
ament_target_dependencies(follow_waypoints
    rclcpp
    rclcpp_action
    nav2_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
)

# Install executable for nav2_to_point_goal
install(TARGETS nav2_to_point_goal
    DESTINATION lib/${PROJECT_NAME}
)

# Install executable for follow_waypoints
install(TARGETS follow_waypoints
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

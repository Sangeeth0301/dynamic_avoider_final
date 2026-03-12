cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacle_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build C++ Goal Node
add_executable(master_goal_node master_goal_node.cpp)
ament_target_dependencies(master_goal_node
  rclcpp
  rclcpp_action
  nav2_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

# Install directories
install(DIRECTORY
  config
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  master_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

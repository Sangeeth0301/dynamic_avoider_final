cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacle_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 1. Compile the C++ master goal node
# (Assuming your master_goal_node.cpp is inside a 'src/' folder)
add_executable(master_goal_node src/master_goal_node.cpp) 
target_include_directories(master_goal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(master_goal_node
  rclcpp rclcpp_action geometry_msgs nav2_msgs tf2 tf2_ros tf2_geometry_msgs)

# 2. Install the C++ executable
install(TARGETS master_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install the Python mission node
# (Assuming mission_node.py is inside a 'scripts/' folder)
install(PROGRAMS
  scripts/mission_node.py
  scripts/odom_tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4. CRITICAL: Install directories (This puts walk.dae into the install space!)
install(DIRECTORY launch config worlds urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
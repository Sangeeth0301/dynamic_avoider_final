cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacle_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (Nav2 and Bridge params)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install Gazebo world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install resource marker for the ROS 2 index
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

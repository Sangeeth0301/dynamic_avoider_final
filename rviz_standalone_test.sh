#!/bin/bash

# RViz Standalone Mock Verifier
# This script simulates a running SUV mission without Gazebo.

# 1. Source the workspace
source install/setup.bash

echo "Starting RViz Standalone Verification..."

# 2. Launch Robot State Publisher (Provides the SUV Shell)
urdf_file="install/dynamic_obstacle_pkg/share/dynamic_obstacle_pkg/resource/super_car.urdf"
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat $urdf_file)" -p use_sim_time:=false &
RSP_PID=$!

# 3. Publish a Mock Transform (Odom -> Base Link)
ros2 run tf2_ros static_transform_publisher 2 0 0 0 0 0 odom base_link &
TF_PID=$!

# 4. Launch RViz with project config
rviz_config="install/dynamic_obstacle_pkg/share/dynamic_obstacle_pkg/config/nav2_default_view.rviz"
ros2 run rviz2 rviz2 -d $rviz_config &
RVIZ_PID=$!

# 5. Publish Mock LaserScan (Green dots ring)
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{
  header: {frame_id: 'super_car/lidar_link/lidar'},
  angle_min: -3.14, angle_max: 3.14, angle_increment: 0.1,
  range_min: 0.1, range_max: 10.0,
  ranges: [5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5]
}" -r 5 &
SCAN_PID=$!

# 6. Publish Mock Path (Cyan line)
ros2 topic pub /plan nav_msgs/msg/Path "{
  header: {frame_id: 'odom'},
  poses: [
    {pose: {position: {x: 0, y: 0, z: 0}}},
    {pose: {position: {x: 1, y: 1, z: 0}}},
    {pose: {position: {x: 2, y: 0, z: 0}}},
    {pose: {position: {x: 4, y: 2, z: 0}}}
  ]
}" -r 1 &
PATH_PID=$!

echo "Verification data is now streaming."
echo "Check RViz for: "
echo " - A blue SUV shell (moved to X=2)"
echo " - A ring of green LiDAR dots"
echo " - A cyan navigation path"
echo ""
echo "Press Ctrl+C to stop the test."

# Cleanup on exit
trap "kill $RSP_PID $TF_PID $RVIZ_PID $SCAN_PID $PATH_PID" SIGINT SIGTERM
wait

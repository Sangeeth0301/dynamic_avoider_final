# Dynamic Obstacle Avoider

Navigate a simulated Ackermann vehicle from **A (-10, 0)** to **B (10, 0)**  
while dynamically avoiding 6 walking actors in Gazebo Harmonic.

**Stack:** ROS2 Jazzy · Gazebo Harmonic · SLAM Toolbox · Nav2 MPPI controller · robot_localization EKF

---

## Requirements

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-nav2-mppi-controller \
  ros-jazzy-nav2-velocity-smoother \
  ros-jazzy-nav2-collision-monitor \
  ros-jazzy-xacro \
  python3-nav2-simple-commander
```

---

## Build

```bash
cd ~/Desktop/dynamic_avoider_final
rm -rf build/ install/ log/
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dynamic_obstacle_pkg
source install/setup.bash
```

---

## Run

```bash
# Single command — starts everything
ros2 launch dynamic_obstacle_pkg total_system.launch.py
```

**Timeline after launch:**
| Time | Event |
|------|-------|
| 0s   | Gazebo loads world with 6 actors |
| 3s   | ROS-GZ bridge connects |
| 5s   | SLAM Toolbox starts mapping |
| 13s  | All Nav2 servers activate |
| 28s  | Mission node sends A→B goal |

The car navigates automatically. Watch the costmap in RViz update in real time as actors walk through it.

---

## Manual goal (optional)

In a second terminal after launch:
```bash
source install/setup.bash
ros2 run dynamic_obstacle_pkg master_goal_node
# Enter: 10 0 0
```

---

## Verification

```bash
source install/setup.bash

# TF tree: map -> odom -> base_footprint -> base_link -> lidar_link
ros2 run tf2_tools view_frames

# EKF output (fused odom + IMU)
ros2 topic echo /odometry/filtered --once

# SLAM map
ros2 topic hz /map

# Nav2 lifecycle state
ros2 lifecycle list
```

---

## File structure

```
src/dynamic_obstacle_pkg/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── ekf.yaml          # robot_localization: fuses /odom + /imu/data_raw
│   └── nav2_params.yaml  # Nav2 + SLAM Toolbox params (all in one file)
├── launch/
│   ├── sim.launch.py           # Gazebo + bridge + RSP + EKF + RViz
│   ├── navigation_launch.py    # SLAM + Nav2 (explicit nodes)
│   └── total_system.launch.py  # Master — run this
├── urdf/
│   └── super_car.urdf.xacro
├── worlds/
│   └── dynamic_world.sdf  # Arena + super_car + 6 walking actors
├── rviz/
│   └── dynamic_avoider.rviz
├── scripts/
│   └── mission_node.py    # Python: nav2_simple_commander A->B
└── src/
    └── master_goal_node.cpp  # C++: interactive goal sender
```

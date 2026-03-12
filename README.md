# Dynamic Obstacle Avoider - Project 36

Navigate from A to B while avoiding moving actors (simulated humans) in Gazebo.

## 🚀 Commands to Run

Open three separate terminals in the project root (`c:\Users\sande\ROS_2\dynamic_avoider_final`):

### Terminal 1: Build and Simulation
```powershell
# Build the workspace
colcon build --symlink-install

# Source and Launch Gazebo + Rviz
call install/setup.bat
ros2 launch dynamic_obstacle_pkg sim_launch.launch.py
```

### Terminal 2: Navigation Stack
```powershell
# Source and Launch SLAM + Nav2
call install/setup.bat
ros2 launch dynamic_obstacle_pkg nav_launch.launch.py
```

### Terminal 3: Git (Optional)
```powershell
# To push changes to your repository
git push origin main
```

## 🛠️ Project Components
- **Gazebo Sim**: 30x30m Arena with 4 central pillars and 6 dynamic actors.
- **Robot**: Ackermann-steered Super Car with LiDAR.
- **Nav2**: Using MPPI Controller for advanced dynamic obstacle avoidance.
- **SLAM**: Real-time mapping using SLAM Toolbox.

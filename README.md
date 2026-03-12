# 🏁 SUV Masterpiece: Dynamic Obstacle Avoider

A state-of-the-art autonomous vehicle simulation featuring **Ackermann Steering**, **MPPI Navigation**, and a **6-Human Central Crowd** in Gazebo Harmonic.

## 🛠️ Technical Stack & Gap Analysis
This project represents a generational leap over standard ROS 2 tutorials:

| Feature | SUV Masterpiece (Our Project) | Reference Standards (Classic) |
| :--- | :--- | :--- |
| **Engine** | **Gazebo Harmonic (Sim)** | Gazebo Classic (Legacy) |
| **ROS Distro** | **ROS 2 Jazzy** (Latest LTS) | ROS 2 Foxy/Humble |
| **Drive Type** | **Ackermann Steering** (Car-like) | Differential Drive (Toy-like) |
| **Physics** | **1ms Precision** | 10ms (Default) |
| **Controller** | **MPPI** (Model Predictive Path Integral) | DWB (Dynamic Window Approach) |

## 🚀 One-Command Launch
We have simplified the execution into a single "Master Key" launch file:

```bash
# 1. Build the workspace
colcon build --packages-select dynamic_obstacle_pkg

# 2. Source and Launch Total System
source install/setup.bash
ros2 launch dynamic_obstacle_pkg total_system.launch.py
```

## 🎯 Interactive Controls
Once the simulation is initialized, you can control the SUV directly through the **C++ Master Goal Controller** in your terminal:
- **Input**: Enter `X Y Yaw` coordinates (e.g., `10.0 5.0 0.0`).
- **Feedback**: Monitor goal status, path planning, and arrival notifications in real-time.

## 📦 Project Architecture
- **worlds/dynamic_world.sdf**: Cinematic environment with 6 dynamic actors and optimized physics.
- **master_goal_node.cpp**: High-performance C++ interactive goal server.
- **urdf/super_car.urdf**: Physical robot description for RViz and Gazebo.
- **config/nav2_default_view.rviz**: Professional high-fidelity visualization.
- **config/nav2_params.yaml**: Tuned MPPI parameters for car-like dynamics.

---
*Elevating Autonomous Simulation to the "Best Possible" State.*

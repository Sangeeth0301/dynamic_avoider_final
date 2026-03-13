"""
total_system.launch.py  —  Master launch
Run this single command to start everything:
  ros2 launch dynamic_obstacle_pkg total_system.launch.py

TIMELINE:
  t= 0s  Gazebo + Bridge(3s) + RSP + Static TF + RViz
  t= 5s  SLAM Toolbox
  t=13s  Nav2 servers  (5s + 8s internal delay)
  t=28s  Mission node  (calls waitUntilNav2Active internally)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("dynamic_obstacle_pkg")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "sim.launch.py")
        )
    )

    nav = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "navigation_launch.py")
            )
        )]
    )

    # Mission node: waitUntilNav2Active() blocks internally until ready
    mission = TimerAction(
        period=28.0,
        actions=[Node(
            package="dynamic_obstacle_pkg",
            executable="mission_node.py",
            name="mission_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )]
    )

    return LaunchDescription([sim, nav, mission])

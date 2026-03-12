"""
total_system.launch.py — Complete rewrite

STARTUP SEQUENCE (timed):
  t=0s   → Gazebo + Bridge + RSP + Static TF + RViz
  t=5s   → SLAM Toolbox + Nav2 (nav_launch handles internal 5s delay for Nav2)
  t=18s  → Master Goal Node (Nav2 needs ~12–15 s to fully activate)

ROOT CAUSE FIXES:
  A. Original launched everything at t=0 simultaneously.
     Gazebo wasn't ready when bridge tried to connect → bridge crash.
     SLAM wasn't ready when Nav2 started → TF tree missing → Nav2 crash.
     Nav2 wasn't ready when goal node started → goals silently dropped.

  B. master_goal_node must NOT be launched via total_system if you want
     interactive stdin. Include it here with output='screen' and it
     will prompt you in the terminal once Nav2 is ready.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dynamic_obstacle_pkg')

    # ──────────────────────────────────────────────────────────
    # 1. Simulation layer  (t = 0 s)
    # ──────────────────────────────────────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'sim_launch.launch.py')
        )
    )

    # ──────────────────────────────────────────────────────────
    # 2. Navigation layer  (t = 5 s)
    #    SLAM needs /scan — must wait for Gazebo + bridge.
    #    nav_launch itself delays Nav2 by another 5 s internally.
    # ──────────────────────────────────────────────────────────
    nav = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'nav_launch.launch.py')
                )
            )
        ]
    )

    # ──────────────────────────────────────────────────────────
    # 3. Master Goal Node  (t = 18 s)
    #    Gives Nav2 time to fully activate all lifecycle nodes.
    #    Will print "Enter Goal Pose [X Y Yaw_rad]:" when ready.
    # ──────────────────────────────────────────────────────────
    goal_node = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='dynamic_obstacle_pkg',
                executable='master_goal_node',
                name='master_goal_node',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        sim,
        nav,
        goal_node,
    ])

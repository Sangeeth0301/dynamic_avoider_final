import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('dynamic_obstacle_pkg')

    # 1. Include Simulation (Gazebo + RViz + Bridge)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sim_launch.launch.py'))
    )

    # 2. Include Navigation (SLAM + Nav2 Stack)
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'nav_launch.launch.py'))
    )

    return LaunchDescription([
        sim,
        nav
    ])

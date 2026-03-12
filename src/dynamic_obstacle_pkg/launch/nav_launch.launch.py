import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('dynamic_obstacle_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # 1. Start SLAM Toolbox (Dynamic Mapping)
    # This builds the costmap from the LiDAR rays in real-time
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map'
        }.items(),
    )

    # 2. Start Navigation Stack (Nav2 Intelligence)
    # Orchestrates controller, planner, and behavior servers
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'autostart': 'True'
        }.items(),
    )

    return LaunchDescription([
        slam,
        nav
    ])

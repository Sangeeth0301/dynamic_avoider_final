import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('dynamic_obstacle_pkg')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # 1. Start SLAM Toolbox (To build a map of your walls live)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items(),
    )

    # 2. Start Navigation Stack (Nav2)
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml')
        }.items(),
    )

    return LaunchDescription([
        slam,
        nav
    ])

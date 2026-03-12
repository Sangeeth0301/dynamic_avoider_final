import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg          = get_package_share_directory('dynamic_obstacle_pkg')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    # ──────────────────────────────────────────────────────────
    # 1. SLAM Toolbox — starts immediately
    # ──────────────────────────────────────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
        }.items(),
    )

    # ──────────────────────────────────────────────────────────
    # 2. Nav2 — delayed 5 s to let SLAM establish TF tree
    # ──────────────────────────────────────────────────────────
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time':                    'True',
                    'params_file':                     nav2_params,
                    'autostart':                       'True',
                    'map_subscribe_transient_local':   'True',
                }.items(),
            )
        ]
    )

    return LaunchDescription([
        slam,
        nav2,
    ])

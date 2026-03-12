"""
sim_launch.launch.py — Complete rewrite

WHAT THIS LAUNCH FILE DOES:
  1. Starts Gazebo Harmonic with dynamic_world.sdf
  2. Starts ros_gz_bridge (after 3s delay for Gazebo to be ready)
  3. Starts robot_state_publisher with the URDF
  4. Publishes a STATIC TF: base_link → lidar_link
     (so Nav2 costmap can transform /scan into base_link frame)
  5. Starts RViz2

ROOT CAUSE FIXES:
  A. Static TF was: base_link → super_car/lidar_link/lidar
     That is WRONG. The SDF sensor uses gz:frame_id=lidar_link.
     The bridge publishes LaserScan with frame_id="lidar_link".
     So the static TF must be: base_link → lidar_link.

  B. Added TimerAction(3s) before bridge so Gazebo topics exist
     before the bridge tries to subscribe to them.

  C. robot_state_publisher reads URDF file with open() —
     this is the correct way; xacro.process_file not needed
     since our URDF has no xacro macros.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('dynamic_obstacle_pkg')

    world_file    = os.path.join(pkg, 'worlds', 'dynamic_world.sdf')
    bridge_config = os.path.join(pkg, 'config',  'bridge_params.yaml')
    rviz_config   = os.path.join(pkg, 'config',  'nav2_default_view.rviz')
    # Use urdf directory where the file actually exists
    urdf_file     = os.path.join(pkg, 'urdf', 'super_car.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ──────────────────────────────────────────────────────────
    # Environment variables
    # Isolate GZ transport to localhost so discovery is instant.
    # ──────────────────────────────────────────────────────────
    env_gz_ip   = SetEnvironmentVariable('GZ_IP',   '127.0.0.1')
    env_gz_part = SetEnvironmentVariable('GZ_PARTITION', 'dynamic_avoider')
    env_ros_disc = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    # ──────────────────────────────────────────────────────────
    # 1. Gazebo Harmonic
    # ──────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ──────────────────────────────────────────────────────────
    # 2. ROS-GZ Bridge (delayed 3 s)
    # ──────────────────────────────────────────────────────────
    bridge = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'config_file': bridge_config,
                         'qos_overrides./tf.publisher.durability': 'transient_local'}],
            output='screen',
        )]
    )

    # ──────────────────────────────────────────────────────────
    # 3. Robot State Publisher
    # Provides: base_link → {wheels, lidar_link} TF from URDF
    # ──────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # ──────────────────────────────────────────────────────────
    # 4. Static TF publisher: base_link → lidar_link
    #    Translation: (0, 0, 1.5) matches URDF lidar_joint origin.
    #    This is what the costmap needs to transform /scan rays.
    # ──────────────────────────────────────────────────────────
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        output='screen',
        arguments=[
            '0', '0', '1.5',   # x y z
            '0', '0', '0', '1', # qx qy qz qw
            'base_link', 'lidar_link'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ──────────────────────────────────────────────────────────
    # 5. RViz2
    # ──────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        env_gz_ip,
        env_gz_part,
        env_ros_disc,
        gazebo,
        bridge,
        robot_state_publisher,
        static_tf_lidar,
        rviz,
    ])

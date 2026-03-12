import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('dynamic_obstacle_pkg')
    world_file = os.path.join(pkg_share, 'worlds', 'dynamic_world.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_default_view.rviz')
    urdf_file = os.path.join(pkg_share, 'resource', 'super_car.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Environment Variables for Industrial Stability
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')
    set_gz_partition = SetEnvironmentVariable('GZ_PARTITION', 'super_car_production')
    set_ros_discovery = SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

    # 1. Gazebo Sim (High-Performance Engine)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. ROS-GZ Bridge (Optimal Topic Mapping)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # 3. Robot State Publisher (URDF Synchronization)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': open(urdf_file).read()
        }],
    )

    # 4. Static TF (Proper Sensor Alignment)
    # Maps Gazebo's nested frame to ROS 2 standard base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '1.2', '0', '0', '0', 'base_link', 'super_car/lidar_link/lidar']
    )

    # 5. Rviz (Visual Masterpiece)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        set_gz_ip,
        set_gz_partition,
        set_ros_discovery,
        gz_sim,
        bridge,
        rsp,
        static_tf,
        rviz
    ])

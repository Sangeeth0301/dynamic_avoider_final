import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dynamic_obstacle_pkg')
    world_file = os.path.join(pkg_share, 'worlds', 'dynamic_world.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_params.yaml')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.15', # CHANGE THIS to 0.15
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'super_car/model/SUV/link/link',
            '--child-frame-id', 'super_car/model/SUV/link/link/lidar'
        ]
    )
    return LaunchDescription([gz_sim, bridge, static_tf])

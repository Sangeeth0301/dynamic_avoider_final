import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'dynamic_obstacle_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    world_file = os.path.join(pkg_share, 'worlds', 'dynamic_world.sdf')
    
    # 1. PATH FIX: Tell Gazebo where the TurtleBot3 models are
    tb3_gazebo_path = get_package_share_directory('turtlebot3_gazebo')
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(tb3_gazebo_path, 'models')
    )

    # 2. Robot State Publisher (URDF)
    urdf_path = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output='screen'
    )

    # 3. Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 4. Spawn Robot - Correct SDF path for Waffle Pi
    model_sdf_path = os.path.join(tb3_gazebo_path, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_waffle_pi',
            '-file', model_sdf_path,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen',
    )

    # 5. The Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_share, 'config', 'bridge_params.yaml')}],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        spawn_robot,
        bridge,
        robot_state_pub
    ])

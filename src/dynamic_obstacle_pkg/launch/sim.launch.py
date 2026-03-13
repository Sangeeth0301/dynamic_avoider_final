import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('dynamic_obstacle_pkg')
    sdf    = os.path.join(pkg, 'worlds', 'dynamic_world.sdf')
    urdf   = os.path.join(pkg, 'urdf',   'super_car.urdf.xacro')
    rviz_f = os.path.join(pkg, 'rviz',   'nav2_view.rviz')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')

    robot_desc = xacro.process_file(urdf).toxml()

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', sdf], output='screen')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

    # Removed /tf bridge to prevent Gazebo from conflicting with EKF
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen')

    # Fuses /odom and /imu/data_raw into a smooth odom -> base_link transform
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_f],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        gazebo,
        rsp,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[ekf]),
        TimerAction(period=2.0, actions=[rviz]),
    ])
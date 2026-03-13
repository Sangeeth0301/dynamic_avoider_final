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

    robot_desc = xacro.process_file(urdf).toxml()

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', sdf], output='screen')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

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
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen')

    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0','0','0','0','0','0','1','map','odom'])

    # Dynamic odom->base_link from actual odometry
    odom_tf_relay = Node(
        package='dynamic_obstacle_pkg',
        executable='odom_tf_relay.py',
        name='odom_tf_relay',
        parameters=[{'use_sim_time': True}],
        output='screen')

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
        static_map_odom,
        TimerAction(period=4.0, actions=[odom_tf_relay]),
        TimerAction(period=2.0, actions=[rviz]),
    ])

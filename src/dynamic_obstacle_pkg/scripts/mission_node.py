#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Set where the robot actually starts in the map
    init = PoseStamped()
    init.header.frame_id = 'map'
    init.header.stamp = nav.get_clock().now().to_msg()
    init.pose.position.x = -10.0
    init.pose.position.y = 0.0
    init.pose.orientation.w = 1.0
    nav.setInitialPose(init)

    nav.get_logger().info('Waiting for Nav2...')
    nav.waitUntilNav2Active(navigator='bt_navigator', localizer='bt_navigator')

    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x = 10.0
    goal.pose.position.y = 0.0
    goal.pose.orientation.w = 1.0

    nav.get_logger().info('Going from (-10,0) to (10,0)...')
    nav.goToPose(goal)

    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb:
            nav.get_logger().info(f'Distance remaining: {fb.distance_remaining:.2f}m')

    nav.get_logger().info(f'Done: {nav.getResult()}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()

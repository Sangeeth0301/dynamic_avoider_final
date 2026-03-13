/**
 * master_goal_node.cpp
 * Interactive C++ goal sender — prompts for X Y Yaw(rad),
 * sends NavigateToPose action goals with feedback logging.
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class MasterGoalNode : public rclcpp::Node
{
public:
  MasterGoalNode() : Node("master_goal_node"), goal_done_(false)
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  bool waitForServer(std::chrono::seconds timeout = std::chrono::seconds(30))
  {
    RCLCPP_INFO(get_logger(), "Waiting for navigate_to_pose action server...");
    if (!client_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(get_logger(), "Server not available after %ld s", timeout.count());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Server ready.");
    return true;
  }

  void sendGoal(double x, double y, double yaw_rad)
  {
    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    goal.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(get_logger(), "Goal -> x=%.2f  y=%.2f  yaw=%.3f rad", x, y, yaw_rad);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    opts.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & h) {
        if (!h) RCLCPP_ERROR(get_logger(), "Goal REJECTED.");
        else     RCLCPP_INFO(get_logger(),  "Goal ACCEPTED.");
      };

    opts.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO(get_logger(), "Distance remaining: %.2f m", fb->distance_remaining);
      };

    opts.result_callback =
      [this](const GoalHandleNav::WrappedResult & r) {
        switch (r.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "SUCCEEDED!"); break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "ABORTED.");  break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(),  "CANCELED.");  break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result.");
        }
        goal_done_ = true;
      };

    goal_done_ = false;
    client_->async_send_goal(goal, opts);

    while (rclcpp::ok() && !goal_done_) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  bool goal_done_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MasterGoalNode>();

  if (!node->waitForServer()) {
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "\n=== Dynamic Obstacle Avoider — Goal Sender ===\n"
            << "  Robot: A(-10, 0)   Default goal: B(10, 0)\n\n";

  while (rclcpp::ok()) {
    std::cout << "Enter goal [X Y Yaw_rad] or 'q': ";
    std::string line;
    if (!std::getline(std::cin, line) || line == "q") break;

    double x = 0, y = 0, yaw = 0;
    if (std::sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &yaw) < 2) {
      std::cerr << "  Usage: X Y [Yaw_rad]\n";
      continue;
    }
    node->sendGoal(x, y, yaw);
  }

  rclcpp::shutdown();
  return 0;
}

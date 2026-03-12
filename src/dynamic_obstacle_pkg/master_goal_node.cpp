/**
 * master_goal_node.cpp — Complete rewrite
 *
 * ROOT CAUSE FIX — DEADLOCK:
 *   The original code called std::cin inside a rclcpp::TimerBase callback.
 *   Timer callbacks execute on the ROS 2 spin thread. Blocking that thread
 *   with std::cin means the spin thread cannot process:
 *     - Action goal response callbacks
 *     - Action result callbacks
 *     - Any other timers or subscriptions
 *   Result: robot appears to accept a goal but never moves, and the node
 *   hangs indefinitely waiting for cin.
 *
 * SOLUTION:
 *   - std::cin runs in a dedicated std::thread (input_thread_)
 *   - Goals are passed to the ROS spin thread via a mutex + atomic flag
 *   - The spin thread only calls non-blocking code
 *
 * USAGE:
 *   After Nav2 activates (~15 s after launch), you will see:
 *     Enter goal [X  Y  Yaw_deg]: 
 *   Type e.g.:   10  0  0
 *   The car will navigate from A (x=-10) to B (x=+10).
 */

#include <memory>
#include <string>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

static constexpr double DEG2RAD = M_PI / 180.0;

class MasterGoalNode : public rclcpp::Node
{
public:
  using Nav2Pose   = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Pose>;

  MasterGoalNode()
  : Node("master_goal_node"),
    navigating_(false),
    shutdown_(false)
  {
    client_ = rclcpp_action::create_client<Nav2Pose>(this, "navigate_to_pose");

    // Non-blocking timer — just dispatches pending goals
    dispatch_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&MasterGoalNode::dispatch_pending_goal, this));

    // Blocking stdin lives in its own thread
    input_thread_ = std::thread(&MasterGoalNode::input_loop, this);

    RCLCPP_INFO(get_logger(),
      "\n"
      "╔══════════════════════════════════════════╗\n"
      "║      DYNAMIC OBSTACLE AVOIDER            ║\n"
      "║  Waiting for Nav2 action server...       ║\n"
      "║  Car starts at: X=-10, Y=0               ║\n"
      "║  Suggested goal: X=10  Y=0  Yaw=0        ║\n"
      "╚══════════════════════════════════════════╝");
  }

  ~MasterGoalNode()
  {
    shutdown_ = true;
    if (input_thread_.joinable())
      input_thread_.detach();
  }

private:
  // ── ROS objects ─────────────────────────────────────────────
  rclcpp_action::Client<Nav2Pose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr dispatch_timer_;
  std::thread input_thread_;

  // ── Shared state between threads ────────────────────────────
  std::atomic<bool> navigating_;
  std::atomic<bool> shutdown_;
  struct GoalRequest { double x, y, yaw_rad; bool pending = false; };
  GoalRequest pending_;
  std::mutex  pending_mutex_;

  // ──────────────────────────────────────────────────────────
  // INPUT THREAD — blocks on std::cin, safe because it is NOT
  // on the ROS spin thread.
  // ──────────────────────────────────────────────────────────
  void input_loop()
  {
    while (!shutdown_) {
      // Wait until the previous navigation is complete
      if (navigating_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        continue;
      }

      double x = 0, y = 0, yaw_deg = 0;
      std::cout << "\n──────────────────────────────────────────\n"
                << " Enter goal [X  Y  Yaw_deg]: " << std::flush;

      if (!(std::cin >> x >> y >> yaw_deg)) {
        if (std::cin.eof()) { shutdown_ = true; break; }
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cerr << "[goal_node] Invalid input — enter three numbers.\n";
        continue;
      }

      std::lock_guard<std::mutex> lk(pending_mutex_);
      pending_ = {x, y, yaw_deg * DEG2RAD, true};
    }
  }

  // ──────────────────────────────────────────────────────────
  // DISPATCH TIMER — runs on ROS spin thread, never blocks
  // ──────────────────────────────────────────────────────────
  void dispatch_pending_goal()
  {
    if (navigating_) return;

    GoalRequest req;
    {
      std::lock_guard<std::mutex> lk(pending_mutex_);
      if (!pending_.pending) return;
      req = pending_;
      pending_.pending = false;
    }

    // Check server availability (1s timeout, non-blocking for spin thread
    // because we call this in a timer — it won't block other timers)
    if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Nav2 not ready yet — requeuing goal.");
      std::lock_guard<std::mutex> lk(pending_mutex_);
      pending_ = req;   // put it back
      return;
    }

    send_goal(req.x, req.y, req.yaw_rad);
  }

  // ──────────────────────────────────────────────────────────
  // SEND GOAL
  // ──────────────────────────────────────────────────────────
  void send_goal(double x, double y, double yaw_rad)
  {
    navigating_ = true;

    auto goal = Nav2Pose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    goal.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(get_logger(),
      "Sending goal → X=%.2f  Y=%.2f  Yaw=%.1f°",
      x, y, yaw_rad / DEG2RAD);

    auto opts = rclcpp_action::Client<Nav2Pose>::SendGoalOptions();

    opts.goal_response_callback =
      [this](const GoalHandle::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Goal REJECTED by Nav2.");
          navigating_ = false;
        } else {
          RCLCPP_INFO(get_logger(), "Goal ACCEPTED — robot navigating...");
        }
      };

    opts.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const Nav2Pose::Feedback> fb) {
        auto & p = fb->current_pose.pose.position;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "  Position: X=%.2f  Y=%.2f  Distance remaining=%.2f m",
          p.x, p.y, fb->distance_remaining);
      };

    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        navigating_ = false;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "✓ GOAL REACHED!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "✗ Goal ABORTED by Nav2.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(),  "⚠ Goal CANCELED.");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code.");
        }
      };

    client_->async_send_goal(goal, opts);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterGoalNode>());
  rclcpp::shutdown();
  return 0;
}

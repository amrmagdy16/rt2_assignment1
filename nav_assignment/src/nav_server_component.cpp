#include <memory>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "nav_interfaces/action/robot_nav.hpp"

namespace nav_assignment
{
class NavServer : public rclcpp::Node
{
public:
  using RobotNav = nav_interfaces::action::RobotNav;
  using GoalHandleRobotNav = rclcpp_action::ServerGoalHandle<RobotNav>;

  explicit NavServer(const rclcpp::NodeOptions & options) : Node("nav_server_node", options)
  {
    // Setup TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Setup Action Server with a ReentrantCallbackGroup for multithreading (allows cancellation!)
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    this->action_server_ = rclcpp_action::create_server<RobotNav>(
      this,
      "robot_nav",
      std::bind(&NavServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavServer::handle_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      cb_group_
    );

    RCLCPP_INFO(this->get_logger(), "NavServer Component Initialized.");
  }

private:
  rclcpp_action::Server<RobotNav>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RobotNav::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request with target x: %f, y: %f", goal->target_x, goal->target_y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRobotNav> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRobotNav> goal_handle)
  {
    // Execute the goal in a separate thread to avoid blocking the executor
    std::thread{std::bind(&NavServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleRobotNav> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<RobotNav::Feedback>();
    auto result = std::make_shared<RobotNav::Result>();

    geometry_msgs::msg::Twist cmd_msg;

    while (rclcpp::ok()) {
      // 1. Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_msg);
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled.");
        return;
      }

      // 2. Look up the robot's current position using TF2
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
        loop_rate.sleep();
        continue;
      }

      double current_x = transformStamped.transform.translation.x;
      double current_y = transformStamped.transform.translation.y;
      
      // Calculate Yaw from Quaternion
      tf2::Quaternion q(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, current_yaw;
      m.getRPY(roll, pitch, current_yaw);

      // 3. Simple Proportional Control Logic
      double distance_error = std::hypot(goal->target_x - current_x, goal->target_y - current_y);
      double angle_to_goal = std::atan2(goal->target_y - current_y, goal->target_x - current_x);
      
      // Normalize angle error to [-pi, pi]
      double angle_error = angle_to_goal - current_yaw;
      while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
      while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

      // Publish feedback
      feedback->distance_remaining = distance_error;
      goal_handle->publish_feedback(feedback);

      // Stop condition: Reached within 0.1 meters
      if (distance_error < 0.1) {
        break;
      }

      // Velocity commands
      if (std::abs(angle_error) > 0.2) {
        // Rotate first
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.5 * angle_error;
      } else {
        // Move forward while adjusting angle slightly
        cmd_msg.linear.x = 0.3 * distance_error;
        // Cap max speed
        if (cmd_msg.linear.x > 0.5) cmd_msg.linear.x = 0.5; 
        cmd_msg.angular.z = 0.5 * angle_error;
      }

      cmd_vel_pub_->publish(cmd_msg);
      loop_rate.sleep();
    }

    // Stop robot on success
    if (rclcpp::ok()) {
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_msg);
      
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
    }
  }
};
}  // namespace nav_assignment

RCLCPP_COMPONENTS_REGISTER_NODE(nav_assignment::NavServer)

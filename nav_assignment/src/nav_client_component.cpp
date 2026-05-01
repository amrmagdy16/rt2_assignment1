#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_interfaces/msg/target_goal.hpp"
#include "nav_interfaces/action/robot_nav.hpp"

namespace nav_assignment
{
class NavClient : public rclcpp::Node
{
public:
  using RobotNav = nav_interfaces::action::RobotNav;
  using GoalHandleRobotNav = rclcpp_action::ClientGoalHandle<RobotNav>;

  explicit NavClient(const rclcpp::NodeOptions & options) : Node("nav_client_node", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<RobotNav>(this, "robot_nav");

    this->ui_subscriber_ = this->create_subscription<nav_interfaces::msg::TargetGoal>(
      "ui_target", 10, std::bind(&NavClient::ui_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "NavClient Component Initialized.");
  }

private:
  rclcpp_action::Client<RobotNav>::SharedPtr client_ptr_;
  rclcpp::Subscription<nav_interfaces::msg::TargetGoal>::SharedPtr ui_subscriber_;

  void ui_callback(const nav_interfaces::msg::TargetGoal::SharedPtr msg)
  {
    if (msg->is_cancel) {
      RCLCPP_WARN(this->get_logger(), "Sending CANCEL request to Action Server...");
      this->client_ptr_->async_cancel_all_goals();
      return;
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = RobotNav::Goal();
    goal_msg.target_x = msg->x;
    goal_msg.target_y = msg->y;
    goal_msg.target_theta = msg->theta;

    RCLCPP_INFO(this->get_logger(), "Forwarding new goal to Action Server...");
    
    auto send_goal_options = rclcpp_action::Client<RobotNav>::SendGoalOptions();
    // Setting up a simple feedback callback to print the remaining distance
    send_goal_options.feedback_callback = 
      [this](GoalHandleRobotNav::SharedPtr, const std::shared_ptr<const RobotNav::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %f", feedback->distance_remaining);
      };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
};
}  // namespace nav_assignment

RCLCPP_COMPONENTS_REGISTER_NODE(nav_assignment::NavClient)

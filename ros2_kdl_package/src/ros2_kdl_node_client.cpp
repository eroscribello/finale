#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


#include "ros2_kdl_package/action/execute_trajectory.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class UnifiedTaskSupervisor : public rclcpp::Node
{
public:
  using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  UnifiedTaskSupervisor() : Node("unified_task_supervisor")
  {
    // action clients
    arm_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa/ExecuteTrajectory");
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "In attesa dei server (Nav2 e Braccio)...");
  }

  // Sending goal for the navigation
  void send_nav_goal(float x, float y)
  {
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 server non disponibile");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Invio base mobile a: X=%.2f, Y=%.2f", x, y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&UnifiedTaskSupervisor::nav_result_callback, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // Callback (navigazione completata)
  void nav_result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "NAVIGAZIONE COMPLETATA! Avvio il braccio...");
      send_arm_goal(1);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigazione fallita o interrotta.");
    }
  }

  // goal for the arm
  void send_arm_goal(int order)
  {
    if (!arm_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Braccio server non disponibile");
      return;
    }

    auto goal_msg = ExecuteTrajectory::Goal();
    goal_msg.order = order;

    auto options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    options.result_callback = [this](const auto & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "BRACCIO COMPLETATO. Missione finita.");
        }
    };

    arm_client_->async_send_goal(goal_msg, options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnifiedTaskSupervisor>();

  //Start mission
  node->send_nav_goal(5.1, 0.3);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

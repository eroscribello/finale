
/*
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_kdl_package/action/execute_trajectory.hpp"

using namespace std::chrono_literals;

class ExecuteTrajectoryClient : public rclcpp::Node
{
public:
  using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
  using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

  ExecuteTrajectoryClient()
  : Node("execute_trajectory_client")
  {
    client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa/ExecuteTrajectory");

    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action not available after 10 seconds");
      rclcpp::shutdown();
    }
  }

  void send_goal(int order)
  {
    auto goal_msg = ExecuteTrajectory::Goal();
    goal_msg.order = order;

    RCLCPP_INFO(this->get_logger(), "Sending Goal: %d", order);

    auto options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
    options.goal_response_callback =
        std::bind(&ExecuteTrajectoryClient::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback =
        std::bind(&ExecuteTrajectoryClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
        std::bind(&ExecuteTrajectoryClient::result_callback, this, std::placeholders::_1);

    client_->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr client_;

  void goal_response_callback(const GoalHandleExecuteTrajectory::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal rifiutato dal server");
    } else {
      RCLCPP_INFO(get_logger(), "Goal accettato, in attesa del risultato...");
    }
  }

  void feedback_callback(
    GoalHandleExecuteTrajectory::SharedPtr,
    const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(),
                "Feedback: position_error = [%f, %f, %f]",
                feedback->position_error[0],
                feedback->position_error[1],
                feedback->position_error[2]);
  }

  void result_callback(const GoalHandleExecuteTrajectory::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Goal Completed Succesfully: %s",
                    result.result->success ? "true" : "false");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal Aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Goal Cancelled");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown Result");
        break;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExecuteTrajectoryClient>();

  // Esempio: invio goal 1
  node->send_goal(1);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
*/


#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Include per il braccio (il tuo)
#include "ros2_kdl_package/action/execute_trajectory.hpp"
// Include per Nav2
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class UnifiedTaskSupervisor : public rclcpp::Node
{
public:
  using ExecuteTrajectory = ros2_kdl_package::action::ExecuteTrajectory;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  UnifiedTaskSupervisor() : Node("unified_task_supervisor")
  {
    // Inizializza i due client
    arm_client_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/iiwa/ExecuteTrajectory");
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "In attesa dei server (Nav2 e Braccio)...");
  }

  // Step 1: Invia il comando alla base mobile
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

  // Callback Risultato Navigazione
  void nav_result_callback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "NAVIGAZIONE COMPLETATA! Avvio il braccio...");
      send_arm_goal(1); // Chiamata al braccio (il tuo codice originale)
    } else {
      RCLCPP_ERROR(this->get_logger(), "Navigazione fallita o interrotta.");
    }
  }

  // Step 2: Invia il comando al braccio (Tua funzione originale leggermente adattata)
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
        // rclcpp::shutdown(); // Opzionale: chiudi tutto alla fine
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

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "interfaces/action/goal_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace parkme
{
class ParkingActionClient : public rclcpp::Node
{
public:
  using Parking = interfaces::action::GoalAction;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Parking>;

  explicit ParkingActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("parking_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Parking>(
      this,
      "parking");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ParkingActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Parking::Goal();
    goal_msg.data = 1;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Parking>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ParkingActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ParkingActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ParkingActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Parking>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Parking::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Feedback Recieved ";
    ss<<feedback->feedback;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: "<<result.result->success;
    
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(parkme::ParkingActionClient)
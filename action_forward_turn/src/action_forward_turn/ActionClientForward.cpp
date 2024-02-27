// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "action_forward_turn/ActionClientForward.hpp"

namespace action_forward_turn
{

using std::placeholders::_1;
using std::placeholders::_2;

ActionClientForward::ActionClientForward()
: Node("action_forward_turn_action_client")
{
  action_client_ = rclcpp_action::create_client<GenerateInformation>(
    this, "generate_information");
}

void
ActionClientForward::send_request(GenerateInformation::Goal goal)
{
  finished_ = false;
  success_ = false;

  if (!action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto send_goal_options =
    rclcpp_action::Client<GenerateInformation>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&ActionClientForward::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&ActionClientForward::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&ActionClientForward::result_callback, this, _1);

  action_client_->async_send_goal(goal, send_goal_options);
}

void
ActionClientForward::goal_response_callback(const GoalHandleGenerateInformation::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
  }
}

void
ActionClientForward::feedback_callback(
  GoalHandleGenerateInformation::SharedPtr,
  const std::shared_ptr<const GenerateInformation::Feedback> feedback)
{
  RCLCPP_INFO(
    get_logger(), "Feedback received: distance to goal = %f", feedback->distance_remaining);
}

void
ActionClientForward::result_callback(const GoalHandleGenerateInformation::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal achieved!!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
  }

  finished_ = true;
  success_ = result.code == rclcpp_action::ResultCode::SUCCEEDED;
}

}  // namespace action_forward_turn

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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_forward_turn/ActionClientTurn.hpp"

namespace action_forward_turn
{
ActionClientTurn::ActionClientTurn()
: Node("action_turn_node")
{

action_client_ = rclcpp_action::create_client<GenerateInformation>(
this, "navigate_to_pose");

}

void
ActionClientTurn::send_turn_request(GenerateInformation::Goal goal)
{
  goal.command = 1;
  goal.value = radians;

  auto send_goal_options = rclcpp_action::Client<GenerateInformation>::SendGoalOptions();

  send_goal_options.goal_response_callback =
  std::bind(&ActionClientTurn::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
  std::bind(&ActionClientTurn::feedback_callback, this, _1, _2);

  action_client_->async_send_goal(goal, send_goal_options);
}

void
ActionClientTurn::goal_response_callback(const GoalHandleGenerateInformation::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
  }
}

void
ActionClientTurn::feedback_callback(
GoalHandleGenerateInformation::SharedPtr,
const std::shared_ptr<const GenerateInformation::Feedback> feedback)
{
  RCLCPP_INFO(
    get_logger(), "Feedback received: radians turned = %f, remaining radians = %f", feedback->current_distance, feedback->remaining_distance);
}

}  // namespace action_forward_turn

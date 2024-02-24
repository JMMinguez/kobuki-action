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

#ifndef ACTION_FORWARD_TURN__ACTIONCLIENTTURN_HPP_
#define ACTION_FORWARD_TURN__ACTIONCLIENTTURN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_forward_turn_interfaces/action/generate_information.hpp"

namespace action_forward_turn
{

using std::placeholders::_1;
using std::placeholders::_2;

class ActionClientTurn : public rclcpp::Node
{
public:
  using GenerateInformation = action_forward_turn_interfaces::action::GenerateInformation;
  using GoalHandleGenerateInformation = rclcpp_action::ClientGoalHandle<GenerateInformation>;
  ActionClientTurn();
  void send_turn_request(GenerateInformation::Goal goal);
  float radians;

private:
  void goal_response_callback(const GoalHandleGenerateInformation::SharedPtr & goal_handle);
  void feedback_callback(
  GoalHandleGenerateInformation::SharedPtr,
  const std::shared_ptr<const GenerateInformation::Feedback> feedback);
  rclcpp_action::Client<GenerateInformation>::SharedPtr action_client_;
};

}  //  action_forward_turn

#endif  // ACTION_FORWARD_TURN__ACTIONCLIENTTURN_HPP_
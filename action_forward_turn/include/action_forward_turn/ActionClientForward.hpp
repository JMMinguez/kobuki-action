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

#ifndef ACTION_FORWARD_TURN__ACTION_CLIENT_HPP_
#define ACTION_FORWARD_TURN__ACTION_CLIENT_HPP_

#include "action_forward_turn_interfaces/action/generate_information.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_forward_turn
{

class ActionClientForward : public rclcpp::Node
{
public:
  using GenerateInformation = action_forward_turn_interfaces::action::GenerateInformation;
  using GoalHandleGenerateInformation = rclcpp_action::ClientGoalHandle<GenerateInformation>;

  ActionClientForward();

  void send_request(GenerateInformation::Goal goal);

  bool is_action_finished() {return finished_;}
  bool is_result_success() {return success_;}

protected:
  virtual void goal_response_callback(const GoalHandleGenerateInformation::SharedPtr & goal_handle);
  virtual void feedback_callback(
    GoalHandleGenerateInformation::SharedPtr,
    const std::shared_ptr<const GenerateInformation::Feedback> feedback);
  virtual void result_callback(const GoalHandleGenerateInformation::WrappedResult & result);

private:
  rclcpp_action::Client<GenerateInformation>::SharedPtr action_client_;
  bool finished_ {false};
  bool success_ {false};
};

}  // namespace action_forward_turn

#endif  // ACTION_FORWARD_TURN__ACTION_CLIENT_HPP_

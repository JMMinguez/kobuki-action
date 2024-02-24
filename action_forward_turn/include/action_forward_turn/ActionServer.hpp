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

#ifndef ACTION_FORWARD_TURN__ACTIONSERVER_HPP_
#define ACTION_FORWARD_TURN__ACTIONSERVER_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"

#include "rclcpp_action/rclcpp_action.hpp"
#include "action_forward_turn_interfaces/action/generate_information.hpp"

namespace action_forward_turn
{

class ActionServer : public rclcpp::Node
{
public:
  using GenerateInformation = action_forward_turn_interfaces::action::GenerateInformation;
  using GoalHandleGenerateInformation = rclcpp_action::ServerGoalHandle<GenerateInformation>;

  ActionServer();
  void start_server();
  double distance;
  bool is_moving;

private:
  void transform_callback();
  void move_forward();
  void go_state(int new_state);
  bool check_distance();

  rclcpp_action::Server<GenerateInformation>::SharedPtr action_server_;
  GenerateInformation::Goal current_goal_;
  std::shared_ptr<GoalHandleGenerateInformation> goal_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  int current_times_;
  rclcpp::TimerBase::SharedPtr timer_pos_check_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_;
  geometry_msgs::msg::Twist l_vel_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GenerateInformation::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGenerateInformation> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleGenerateInformation> goal_handle);

  void execute();

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  tf2::Stamped<tf2::Transform> odom2bf_;
  tf2::Transform odom2bf_inverse;

  bool start_;
  double actual_distance_;

  const float MOVE_SPEED = 0.3;
  const float STOP_SPEED = 0.0;

  static const int FORWARD = 0;
  static const int TURN = 1;
  int state_;
};

}  // action_forward_turn

#endif  // COMMS__ACTION_SERVER_HPP_
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

#include <chrono>

#include "action_forward_turn/ActionServer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

#include "action_forward_turn_interfaces/action/generate_information.hpp"

namespace action_forward_turn
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

ActionServer::ActionServer()
: Node("action_server_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  is_moving(true)
{
  timer_pos_check_ = create_wall_timer(
    50ms, std::bind(&ActionServer::transform_callback, this));
  
  vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void
ActionServer::start_server()
{
  action_server_ = rclcpp_action::create_server<GenerateInformation>(
    shared_from_this(),
    "tu_accion",
    std::bind(&ActionServer::handle_goal, this, _1, _2),
    std::bind(&ActionServer::handle_cancel, this, _1),
    std::bind(&ActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "Action Server Ready.");
}

rclcpp_action::GoalResponse
ActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GenerateInformation::Goal> goal)
{
  (void)uuid;

  if (goal->command == 0) {
    go_state(FORWARD);
  } else if (goal->command == 1) {
    go_state(TURN);
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleGenerateInformation> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  timer_ = nullptr;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ActionServer::handle_accepted(std::shared_ptr<GoalHandleGenerateInformation> goal_handle)
{
  
  goal_handle_ = goal_handle;
  current_times_ = 0;
  timer_ = create_wall_timer(
    1s, std::bind(&ActionServer::execute, this));
}

void
ActionServer::execute()
{
  if (current_times_ < 10 && !goal_handle_->is_canceling()) {
    auto feedback = std::make_shared<GenerateInformation::Feedback>();

    feedback->current_distance = current_times_;
    feedback->remaining_distance = 10.0 - current_times_;
    goal_handle_->publish_feedback(feedback);
  } else {
    auto result = std::make_shared<GenerateInformation::Result>();

    if (goal_handle_->is_canceling()) {
      goal_handle_->canceled(result);

      RCLCPP_INFO(get_logger(), "Action Canceled");
    } else if (current_times_ >= 10) {
      goal_handle_->succeed(result);
      RCLCPP_INFO(get_logger(), "Action Succeeded");
    }

    timer_ = nullptr;
  }
  current_times_++;
}

void
ActionServer::transform_callback()
{
  tf2::Stamped<tf2::Transform> odom2bfa;
  std::string error;

  if (start_) {
    if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
      auto odom2bf_msg = tf_buffer_.lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero);
      tf2::fromMsg(odom2bf_msg, odom2bf_);
    }

    odom2bf_inverse = odom2bf_.inverse();
    start_ = false;
  }
  // Gets the tf from start 'odom' and actual 'base_footprint'
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bfa_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    tf2::fromMsg(odom2bfa_msg, odom2bfa);

    // Gets the tf from start 'base_footprint' and actual 'base_footprint'
    tf2::Transform bf2bfa = odom2bf_inverse * odom2bfa;

    std::cerr << "BF: \t" << odom2bf_.getOrigin().x() << " " << odom2bf_.getOrigin().y() <<
      std::endl;
    std::cerr << "BFa: \t" << odom2bfa.getOrigin().x() << " " << odom2bfa.getOrigin().y() <<
      std::endl;
    std::cerr << "TF: \t" << bf2bfa.getOrigin().x() << " " << bf2bfa.getOrigin().y() << std::endl;

    //  Extracts the x and y coordinates from the obtained transformation.
    double x = bf2bfa.getOrigin().x();
    double y = bf2bfa.getOrigin().y();

    //  Calculate the distance between (0,0) and (x,y)
    actual_distance_ = sqrt(x * x + y * y);
  }
}

void
ActionServer::move_forward()
{
  //  FSM
  switch (state_) {
    case FORWARD:
      RCLCPP_INFO(get_logger(), "Moving forward!");
      l_vel_.linear.x = MOVE_SPEED;
      vel_->publish(l_vel_);
      is_moving = true;

      if (check_distance()) {
        go_state(TURN);
      }
      break;

    case TURN:
      RCLCPP_INFO(get_logger(), "STOP!");
      l_vel_.linear.x = STOP_SPEED;
      vel_->publish(l_vel_);
      is_moving = false;

      if (!check_distance()) {
        go_state(FORWARD);
      }
      break;
  }
}

void
ActionServer::go_state(int new_state)
{
  //  Change state
  state_ = new_state;
}

bool
ActionServer::check_distance()
{
  //  Check distance
  return actual_distance_ >= distance;
}

}  // namespace comms
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

#include "action_forward_turn/ActionServer.hpp"

namespace action_forward_turn
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


ActionServer::ActionServer()
: Node("action_forward_turn_action_server"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  action_server_ = rclcpp_action::create_server<GenerateInformation>(
    this, "generate_information",
    std::bind(&ActionServer::handle_goal, this, _1, _2),
    std::bind(&ActionServer::handle_cancel, this, _1),
    std::bind(&ActionServer::handle_accepted, this, _1));

  vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

void
ActionServer::start_server()
{
  RCLCPP_INFO(get_logger(), "Action Server Ready.");
}

rclcpp_action::GoalResponse
ActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const GenerateInformation::Goal> goal)
{
  start_time_initialized_ = false;
  start_ = true;
  once_ = true;
  RCLCPP_INFO(
    get_logger(), "Received goal with command %d and distance %f", goal->command, goal->distance);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleGenerateInformation> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ActionServer::handle_accepted(const std::shared_ptr<GoalHandleGenerateInformation> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Goal accepted by server");
  goal_handle_ = goal_handle;
  current_times_ = 0;
  timer_ = create_wall_timer(
    334ms, std::bind(&ActionServer::execute, this));
}

void
ActionServer::execute()
{
  // RCLCPP_INFO(get_logger(), "EXECUTE");

  if (!goal_handle_->is_canceling()) {
    auto feedback = std::make_shared<GenerateInformation::Feedback>();

    transform_callback();

    remaining_distance = goal_handle_->get_goal()->distance - actual_distance_;
    // std::cerr << "Distnace: \t" << goal_handle_->get_goal()->distance << std::endl;
    // std::cerr << "RD: \t" << remaining_distance << std::endl;

    if (!start_time_initialized_) {
      start_time_ = std::chrono::steady_clock::now();
      start_time_initialized_ = true;
    }

    if (remaining_distance <= 0) {
      m_vel_.linear.x = 0;
      m_vel_.angular.z = 0;
      vel_->publish(m_vel_);

      auto elapsed_time = std::chrono::steady_clock::now() - start_time_;
      auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed_time).count();

      if (once_) {
        std::cerr << "Total time: \t" << elapsed_seconds << " seconds" << std::endl;
        std::cerr << "Total distance: \t" << actual_distance_ << std::endl;
        once_ = false;
      }

    } else {
      feedback->current_distance = actual_distance_;
      goal_handle_->publish_feedback(feedback);
      feedback->distance_remaining = remaining_distance;
      goal_handle_->publish_feedback(feedback);

      if (goal_handle_->get_goal()->command == 0) { // Si el comando es 0, el robot se mueve hacia adelante
        m_vel_.linear.x = 0.3;
        m_vel_.angular.z = 0;

      } else if (goal_handle_->get_goal()->command == 1) { // Si el comando es 1, el robot gira
        m_vel_.linear.x = 0;
        m_vel_.angular.z = 0.3;
      }
      vel_->publish(m_vel_);
    }

  } else {
    auto result = std::make_shared<GenerateInformation::Result>();

    if (goal_handle_->is_canceling()) {
      goal_handle_->canceled(result);

      RCLCPP_INFO(get_logger(), "Action Canceled");
    } else if (remaining_distance <= 0) {
      goal_handle_->succeed(result);
      RCLCPP_INFO(get_logger(), "Navigation Succeeded");
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

    //  Extracts the x and y coordinates from the obtained transformation.
    double x = bf2bfa.getOrigin().x();
    double y = bf2bfa.getOrigin().y();

    // std::cerr << "command: \t" << goal_handle_->get_goal()->command << std::endl;

    if (goal_handle_->get_goal()->command == 0) { // Si el comando es 0, el robot se mueve hacia adelante
      //  Calculate the distance between (0,0) and (x,y)
      actual_distance_ = sqrt(x * x + y * y);

    } else if (goal_handle_->get_goal()->command == 1) { // Si el comando es 1, el robot gira
      //  Calculate the angle between (0,0) and (x,y)
      tf2::Matrix3x3 mat(bf2bfa.getRotation());
      mat.getRPY(roll_, pitch_, yaw_);
      actual_distance_ = yaw_;
    }

    // std::cerr << "AD: \t" << actual_distance_ << std::endl;

  }
}

}  // namespace action_forward_turn

#ifndef ACTION_FORWARD_TURN__ACTION_SERVER_HPP_
#define ACTION_FORWARD_TURN__ACTION_SERVER_HPP_

#include "action_forward_turn_interfaces/action/generate_information.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "rclcpp_action/rclcpp_action.hpp"

namespace action_forward_turn
{

class ActionServer : public rclcpp::Node
{
public:
  using GenerateInformation = action_forward_turn_interfaces::action::GenerateInformation;
  using GoalHandleGenerateInformation = rclcpp_action::ServerGoalHandle<GenerateInformation>;

  ActionServer();

  void start_server();

private:
  void transform_callback();
  void execute();

  rclcpp_action::Server<GenerateInformation>::SharedPtr action_server_;
  bool finished_ {false};
  bool success_ {false};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GenerateInformation::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGenerateInformation> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGenerateInformation> goal_handle);
  rclcpp::TimerBase::SharedPtr timer_;
  int current_times_;
  std::shared_ptr<GoalHandleGenerateInformation> goal_handle_;

  rclcpp::TimerBase::SharedPtr timer_pos_check_;
  rclcpp::TimerBase::SharedPtr timer_publish_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  tf2::Stamped<tf2::Transform> odom2bf_;
  tf2::Transform odom2bf_inverse;

  double actual_distance_;
  bool start_;

  double roll_, pitch_, yaw_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_;
  geometry_msgs::msg::Twist m_vel_;
};

}  // namespace action_forward_turn

#endif  // ACTION_FORWARD_TURN__ACTION_SERVER_HPP_


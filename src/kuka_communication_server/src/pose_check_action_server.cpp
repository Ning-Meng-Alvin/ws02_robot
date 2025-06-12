#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <kuka_interfaces/action/check_pose.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

using CheckPose = kuka_interfaces::action::CheckPose;
using GoalHandleCheckPose = rclcpp_action::ServerGoalHandle<CheckPose>;

class PoseCheckActionServer : public rclcpp::Node {
public:
  PoseCheckActionServer()
  : Node("pose_check_action_server_node"),
    move_group_(std::shared_ptr<rclcpp::Node>(this), "kr210_arm")
  {
    this->declare_parameter<std::string>("end_effector", "camera_position");
    this->get_parameter("end_effector", end_eff_);

    move_group_.setEndEffectorLink(end_eff_);

    action_server_ = rclcpp_action::create_server<CheckPose>(
      this,
      "pose_check_action",
      std::bind(&PoseCheckActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PoseCheckActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PoseCheckActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "pose_check_action 服务端已启动");
  }

private:
  std::string end_eff_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp_action::Server<CheckPose>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CheckPose::Goal> goal)
  {
    if (goal->poses.empty()) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCheckPose>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCheckPose> goal_handle)
  {
    std::thread{std::bind(&PoseCheckActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCheckPose> gh)
  {
    const auto goal = gh->get_goal();
    move_group_.setEndEffectorLink(goal->end_effector);
    auto result = std::make_shared<CheckPose::Result>();
    auto feedback = std::make_shared<CheckPose::Feedback>();
    size_t success_count = 0;
    size_t total = goal->poses.size();

    for (size_t i = 0; i < total; ++i)
    {
      if (gh->is_canceling()) {
        gh->canceled(result);
        return;
      }

      auto pose = goal->poses[i];

      move_group_.setPoseTarget(pose.pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      result->success.push_back(success);

      if (success) ++success_count;

      feedback->current_index = i + 1;
      feedback->total = total;

      gh->publish_feedback(feedback);
    }

    result->overall_success_rate = static_cast<float>(success_count) / total;
    gh->succeed(result);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseCheckActionServer>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <kuka_interfaces/action/cartesian_follow.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <thread>

class CartesianFollowServer : public rclcpp::Node
{
public:
  using Follow = kuka_interfaces::action::CartesianFollow;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Follow>;

  CartesianFollowServer() : Node("cartesian_follow_server") {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<Follow>(
      this, "cartesian_follow",
      std::bind(&CartesianFollowServer::handle_goal, this, _1, _2),
      std::bind(&CartesianFollowServer::handle_cancel, this, _1),
      std::bind(&CartesianFollowServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Follow>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "接收到目标，点数：%ld", goal->path.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "接收到取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() {
      execute(goal_handle);
    }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Follow::Feedback>();
    auto result = std::make_shared<Follow::Result>();

    rclcpp::NodeOptions opts;
    auto move_node = std::make_shared<rclcpp::Node>("moveit_tmp_node", opts);
    moveit::planning_interface::MoveGroupInterface move_group(move_node, "kr210_arm");
    move_group.setEndEffectorLink(goal->end_effector);

    std::vector<geometry_msgs::msg::Pose> transformed_poses;
    for (const auto & pose : goal->path) {
      geometry_msgs::msg::Pose new_pose = pose;

      Eigen::Vector3d T(1.0, 0.0, 0.0);
      Eigen::Matrix3d R;
      R << -1, 0, 0,
            0, -1, 0,
            0,  0, 1;
      Eigen::Vector3d p_world(pose.position.x, pose.position.y, pose.position.z);
      Eigen::Vector3d p_fp = R.transpose() * (p_world - T);
      new_pose.position.x = p_fp(0);
      new_pose.position.y = p_fp(1);
      new_pose.position.z = p_fp(2);

      tf2::Quaternion q_input(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w);
      tf2::Quaternion q_tf;
      q_tf.setRPY(0, 0, M_PI);
      tf2::Quaternion q_fp = q_tf.inverse() * q_input;
      new_pose.orientation = tf2::toMsg(q_fp);

      transformed_poses.push_back(new_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = move_group.computeCartesianPath(
      transformed_poses, eef_step, jump_threshold, trajectory);

    feedback->progress = static_cast<float>(fraction);
    goal_handle->publish_feedback(feedback);

    if (fraction < 1.0) {
      result->success = false;
      result->message = "规划未完全成功，fraction = " + std::to_string(fraction);
      goal_handle->succeed(result);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    move_group.execute(plan);

    result->success = true;
    result->message = "笛卡尔路径执行完成";
    goal_handle->succeed(result);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianFollowServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

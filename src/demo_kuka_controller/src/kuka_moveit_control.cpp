#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kuka_moveit_controller_node");

    // 声明参数
    node->declare_parameter<std::string>("plangroup", "camera_position");
    node->declare_parameter<double>("x", -0.782);
    node->declare_parameter<double>("y", 0.0);
    node->declare_parameter<double>("z", 1.584);
    node->declare_parameter<double>("qx", 0.0);
    node->declare_parameter<double>("qy", 0.0);
    node->declare_parameter<double>("qz", 0.0);
    node->declare_parameter<double>("qw", 1.0);

    // 读取参数
    std::string end_effector;
    double x, y, z, qx, qy, qz, qw;
    node->get_parameter("plangroup", end_effector);
    node->get_parameter("x", x);
    node->get_parameter("y", y);
    node->get_parameter("z", z);
    node->get_parameter("qx", qx);
    node->get_parameter("qy", qy);
    node->get_parameter("qz", qz);
    node->get_parameter("qw", qw);

    // 创建 MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(node, "kr210_arm");
    move_group.setEndEffectorLink(end_effector);

    RCLCPP_INFO(node->get_logger(), "Using end effector: %s", end_effector.c_str());

    // 构造 world 坐标下目标 pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    // 坐标系变换 world → footprint
    Eigen::Vector3d T(1.0, 0.0, 0.0);
    Eigen::Matrix3d R;
    R << -1, 0, 0,
          0, -1, 0,
          0,  0, 1;

    Eigen::Vector3d p_world(x, y, z);
    Eigen::Vector3d p_fp = R.transpose() * (p_world - T);
    target_pose.position.x = p_fp(0);
    target_pose.position.y = p_fp(1);
    target_pose.position.z = p_fp(2);

    tf2::Quaternion q_input(qx, qy, qz, qw);
    tf2::Quaternion q_tf;
    q_tf.setRPY(0, 0, M_PI);  // world → footprint 的旋转
    tf2::Quaternion q_fp = q_tf.inverse() * q_input;
    target_pose.orientation = tf2::toMsg(q_fp);

    RCLCPP_INFO(node->get_logger(), "Pose in kuka_footprint: x=%.3f y=%.3f z=%.3f",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Planning %s", success ? "SUCCEEDED" : "FAILED");

    if (success) {
        move_group.execute(my_plan);
    }

    rclcpp::shutdown();
    return 0;
}
// === cartesian_follow_action_server.cpp ===
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

    for (size_t i = 0; i < goal->path.size(); ++i) {
      auto pose = goal->path[i];

      Eigen::Vector3d T(1.0, 0.0, 0.0);
      Eigen::Matrix3d R;
      R << -1, 0, 0,
            0, -1, 0,
            0,  0, 1;
      Eigen::Vector3d p_world(pose.position.x, pose.position.y, pose.position.z);
      Eigen::Vector3d p_fp = R.transpose() * (p_world - T);
      pose.position.x = p_fp(0);
      pose.position.y = p_fp(1);
      pose.position.z = p_fp(2);

      tf2::Quaternion q_input(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w);
      tf2::Quaternion q_tf;
      q_tf.setRPY(0, 0, M_PI);
      tf2::Quaternion q_fp = q_tf.inverse() * q_input;
      pose.orientation = tf2::toMsg(q_fp);

      move_group.setPoseTarget(pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
      }

      feedback->progress = static_cast<float>(i + 1) / goal->path.size();
      goal_handle->publish_feedback(feedback);
    }

    result->success = true;
    result->message = "执行完成";
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

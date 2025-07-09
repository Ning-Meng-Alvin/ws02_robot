// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <kuka_interfaces/action/cartesian_follow.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <Eigen/Dense>
// #include <thread>

// class CartesianFollowServer : public rclcpp::Node
// {
// public:
//   using Follow = kuka_interfaces::action::CartesianFollow;
//   using GoalHandle = rclcpp_action::ServerGoalHandle<Follow>;

//   CartesianFollowServer() : Node("cartesian_follow_server") {
//     using namespace std::placeholders;
//     server_ = rclcpp_action::create_server<Follow>(
//       this, "cartesian_follow",
//       std::bind(&CartesianFollowServer::handle_goal, this, _1, _2),
//       std::bind(&CartesianFollowServer::handle_cancel, this, _1),
//       std::bind(&CartesianFollowServer::handle_accepted, this, _1));
//   }

// private:
//   rclcpp_action::Server<Follow>::SharedPtr server_;

//   rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> goal) {
//     RCLCPP_INFO(this->get_logger(), "接收到目标，点数：%ld", goal->path.size());
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
//     RCLCPP_INFO(this->get_logger(), "接收到取消请求");
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }

//   void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
//     std::thread([this, goal_handle]() {
//       execute(goal_handle);
//     }).detach();
//   }

//   void execute(const std::shared_ptr<GoalHandle> goal_handle) {
//     auto goal = goal_handle->get_goal();
//     auto feedback = std::make_shared<Follow::Feedback>();
//     auto result = std::make_shared<Follow::Result>();

//     rclcpp::NodeOptions opts;
//     auto move_node = std::make_shared<rclcpp::Node>("moveit_tmp_node", opts);
//     moveit::planning_interface::MoveGroupInterface move_group(move_node, "kr210_arm");
//     move_group.setEndEffectorLink(goal->end_effector);

//     std::vector<geometry_msgs::msg::Pose> transformed_poses;
//     for (const auto & pose : goal->path) {
//       geometry_msgs::msg::Pose new_pose = pose;

//       Eigen::Vector3d T(1.0, 0.0, 0.0);
//       Eigen::Matrix3d R;
//       R << -1, 0, 0,
//             0, -1, 0,
//             0,  0, 1;
//       Eigen::Vector3d p_world(pose.position.x, pose.position.y, pose.position.z);
//       Eigen::Vector3d p_fp = R.transpose() * (p_world - T);
//       new_pose.position.x = p_fp(0);
//       new_pose.position.y = p_fp(1);
//       new_pose.position.z = p_fp(2);

//       tf2::Quaternion q_input(
//         pose.orientation.x, pose.orientation.y,
//         pose.orientation.z, pose.orientation.w);
//       tf2::Quaternion q_tf;
//       q_tf.setRPY(0, 0, M_PI);
//       tf2::Quaternion q_fp = q_tf.inverse() * q_input;
//       new_pose.orientation = tf2::toMsg(q_fp);

//       transformed_poses.push_back(new_pose);
//     }

//     // 先移动到第一个点（起点）
//     geometry_msgs::msg::Pose start_pose = transformed_poses.front();
//     move_group.setPoseTarget(start_pose);
//     moveit::planning_interface::MoveGroupInterface::Plan start_plan;
//     bool success = (move_group.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (!success) {
//       result->success = false;
//       result->message = "无法移动到轨迹起点";
//       goal_handle->succeed(result);
//       return;
//     }

//     move_group.execute(start_plan);
//     rclcpp::sleep_for(std::chrono::milliseconds(500));  // 停顿一会再继续

//     // 再规划整个笛卡尔路径
//     moveit_msgs::msg::RobotTrajectory trajectory;
//     const double eef_step = 0.01;
//     const double jump_threshold = 0.0;

//     double fraction = move_group.computeCartesianPath(
//       transformed_poses, eef_step, jump_threshold, trajectory);

//     feedback->progress = static_cast<float>(fraction);
//     goal_handle->publish_feedback(feedback);

//     if (fraction < 1.0) {
//       result->success = false;
//       result->message = "⚠️ 规划未完全成功,fraction = " + std::to_string(fraction);
//       goal_handle->succeed(result);
//       return;
//     }

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     plan.trajectory_ = trajectory;
//     move_group.execute(plan);

//     result->success = true;
//     result->message = "笛卡尔路径执行完成";
//     goal_handle->succeed(result);
//   }
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<CartesianFollowServer>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }






#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <kuka_interfaces/action/cartesian_follow.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <thread>
#include <limits>

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
    RCLCPP_INFO(this->get_logger(), "Received goal with %ld poses", goal->path.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread([this, goal_handle]() {
      execute(goal_handle);
    }).detach();
  }

  std::vector<geometry_msgs::msg::Pose> generate_orientation_variants(const geometry_msgs::msg::Pose& pose, double degree_range = 4.0) {
    std::vector<geometry_msgs::msg::Pose> variants;
    double rad = degree_range * M_PI / 180.0;
    std::vector<double> angles = {-rad, 0.0, rad};
    tf2::Quaternion base_q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    for (double droll : angles) {
      for (double dpitch : angles) {
        for (double dyaw : angles) {
          tf2::Quaternion dq;
          dq.setRPY(droll, dpitch, dyaw);
          tf2::Quaternion q_new = dq * base_q;
          q_new.normalize();
          geometry_msgs::msg::Pose p = pose;
          p.orientation = tf2::toMsg(q_new);
          variants.push_back(p);
        }
      }
    }
    return variants;
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
      

      tf2::Quaternion q_input(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      tf2::Quaternion q_tf;
      q_tf.setRPY(0.0,0.0, M_PI);
      // q_tf.setRPY(0.0,0.0, 0.0);
      tf2::Quaternion q_fp = q_tf.inverse() * q_input;
      new_pose.orientation = tf2::toMsg(q_fp);
      // new_pose.orientation = tf2::toMsg(q_tf);

      transformed_poses.push_back(new_pose);
    }

    // 姿态容忍 ±4°，并记录选中变体
    bool success = false;
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    geometry_msgs::msg::Pose used_variant;
    auto variants = generate_orientation_variants(transformed_poses.front());
    for (const auto& variant : variants) {
      move_group.setPoseTarget(variant);
      if (move_group.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        used_variant = variant;
        success = true;
        break;
      }
    }

    if (!success) {
      result->success = false;
      result->message = "Failed to move to starting pose with relaxed orientation";
      goal_handle->succeed(result);
      return;
    }

    // 插值最短旋转路径
    tf2::Quaternion q_start, q_goal, q_interp;
    tf2::fromMsg(used_variant.orientation, q_start);
    for (size_t i = 0; i < transformed_poses.size(); ++i) {
      double t = static_cast<double>(i) / (transformed_poses.size() - 1);
      tf2::fromMsg(transformed_poses[i].orientation, q_goal);
      q_interp = q_start.slerp(q_goal, t);
      transformed_poses[i].orientation = tf2::toMsg(q_interp);
    }

    move_group.execute(start_plan);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    const int segment_size = 1;
    const double eef_step = 0.2;
    const double jump_threshold = 0.0;
    int executed_segments = 0;
    int total_segments = (transformed_poses.size() + segment_size - 1) / segment_size;

    for (size_t i = 0; i < transformed_poses.size(); i += segment_size) {
      auto start_it = transformed_poses.begin() + i;
      auto end_it = (i + segment_size < transformed_poses.size()) ? start_it + segment_size : transformed_poses.end();
      std::vector<geometry_msgs::msg::Pose> segment(start_it, end_it);
      moveit_msgs::msg::RobotTrajectory traj;
      double fraction = move_group.computeCartesianPath(segment, eef_step, jump_threshold, traj);

      feedback->progress = float(i + segment.size()) / transformed_poses.size();
      goal_handle->publish_feedback(feedback);

      if (fraction < 0.9) {
        RCLCPP_WARN(this->get_logger(), "Segment %d/%d failed (fraction=%.3f), skipping...", executed_segments + 1, total_segments, fraction);
        continue;
      }

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = traj;
      move_group.execute(plan);
      executed_segments++;
    }

    result->success = true;
    result->message = "Segmented Cartesian execution complete";
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












// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <kuka_interfaces/action/cartesian_follow.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <Eigen/Dense>
// #include <thread>

// class CartesianFollowServer : public rclcpp::Node
// {
// public:
//   using Follow = kuka_interfaces::action::CartesianFollow;
//   using GoalHandle = rclcpp_action::ServerGoalHandle<Follow>;

//   CartesianFollowServer() : Node("cartesian_follow_server") {
//     using namespace std::placeholders;
//     server_ = rclcpp_action::create_server<Follow>(
//       this, "cartesian_follow",
//       std::bind(&CartesianFollowServer::handle_goal, this, _1, _2),
//       std::bind(&CartesianFollowServer::handle_cancel, this, _1),
//       std::bind(&CartesianFollowServer::handle_accepted, this, _1));
//   }

// private:
//   rclcpp_action::Server<Follow>::SharedPtr server_;

//   rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> goal) {
//     RCLCPP_INFO(this->get_logger(), "Received goal with %ld poses", goal->path.size());
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
//     RCLCPP_INFO(this->get_logger(), "Cancel request received");
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }

//   void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
//     std::thread([this, goal_handle]() {
//       execute(goal_handle);
//     }).detach();
//   }

//   void execute(const std::shared_ptr<GoalHandle> goal_handle) {
//     auto goal = goal_handle->get_goal();
//     auto feedback = std::make_shared<Follow::Feedback>();
//     auto result = std::make_shared<Follow::Result>();

//     rclcpp::NodeOptions opts;
//     auto move_node = std::make_shared<rclcpp::Node>("moveit_tmp_node", opts);
//     moveit::planning_interface::MoveGroupInterface move_group(move_node, "kr210_arm");
//     move_group.setEndEffectorLink(goal->end_effector);

//     std::vector<geometry_msgs::msg::Pose> transformed_poses;
//     for (const auto & pose : goal->path) {
//       geometry_msgs::msg::Pose new_pose = pose;

//       Eigen::Vector3d T(1.0, 0.0, 0.0);
//       Eigen::Matrix3d R;
//       R << -1, 0, 0,
//             0, -1, 0,
//             0,  0, 1;
//       Eigen::Vector3d p_world(pose.position.x, pose.position.y, pose.position.z);
//       Eigen::Vector3d p_fp = R.transpose() * (p_world - T);
//       new_pose.position.x = p_fp(0);
//       new_pose.position.y = p_fp(1);
//       new_pose.position.z = p_fp(2);

//       tf2::Quaternion q_input(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
//       tf2::Quaternion q_tf;
//       q_tf.setRPY(0, 0, M_PI);
//       tf2::Quaternion q_fp = q_tf.inverse() * q_input;
//       new_pose.orientation = tf2::toMsg(q_fp);

//       transformed_poses.push_back(new_pose);
//     }

//     // 设定 seed 为当前状态，跳过软限位搜索
//     move_group.setPoseTarget(transformed_poses.front());
//     move_group.setStartStateToCurrentState();
//     moveit::planning_interface::MoveGroupInterface::Plan start_plan;
//     bool success = (move_group.plan(start_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if (!success) {
//       result->success = false;
//       result->message = "Failed to move to start pose (TRAC-IK)";
//       goal_handle->succeed(result);
//       return;
//     }

//     move_group.execute(start_plan);
//     rclcpp::sleep_for(std::chrono::milliseconds(500));

//     const int segment_size = 1;
//     const double eef_step = 0.2;
//     const double jump_threshold = 0.0;
//     int executed_segments = 0;
//     int total_segments = (transformed_poses.size() + segment_size - 1) / segment_size;

//     for (size_t i = 0; i < transformed_poses.size(); i += segment_size) {
//       auto start_it = transformed_poses.begin() + i;
//       auto end_it = (i + segment_size < transformed_poses.size()) ? start_it + segment_size : transformed_poses.end();
//       std::vector<geometry_msgs::msg::Pose> segment(start_it, end_it);

//       // 为每段设置 seed
//       move_group.setStartStateToCurrentState();

//       moveit_msgs::msg::RobotTrajectory traj;
//       double fraction = move_group.computeCartesianPath(segment, eef_step, jump_threshold, traj);

//       feedback->progress = float(i + segment.size()) / transformed_poses.size();
//       goal_handle->publish_feedback(feedback);

//       if (fraction < 0.9) {
//         RCLCPP_WARN(this->get_logger(), "Segment %d/%d failed (fraction=%.3f), skipping...", executed_segments + 1, total_segments, fraction);
//         continue;
//       }

//       moveit::planning_interface::MoveGroupInterface::Plan plan;
//       plan.trajectory_ = traj;
//       move_group.execute(plan);
//       executed_segments++;
//     }

//     result->success = true;
//     result->message = "Segmented Cartesian execution complete (with seed IK)";
//     goal_handle->succeed(result);
//   }
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<CartesianFollowServer>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

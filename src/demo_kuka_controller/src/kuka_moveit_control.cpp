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

    // // 构造 world 坐标下目标 pose
    // geometry_msgs::msg::Pose target_pose;
    // target_pose.position.x = x;
    // target_pose.position.y = y;
    // target_pose.position.z = z;

    // target_pose.orientation.x = qx;
    // target_pose.orientation.y = qy;
    // target_pose.orientation.z = qz;
    // target_pose.orientation.w = qw;

    // // 坐标系变换 world → footprint
    // Eigen::Vector3d T(1.0, 0.0, 0.0);
    // Eigen::Matrix3d R;
    // R << -1, 0, 0,
    //       0, -1, 0,
    //       0,  0, 1;

    // Eigen::Vector3d p_world(x, y, z);
    // Eigen::Vector3d p_fp = R.transpose() * (p_world - T);
    // target_pose.position.x = p_fp(0);
    // target_pose.position.y = p_fp(1);
    // target_pose.position.z = p_fp(2);

    // tf2::Quaternion q_input(qx, qy, qz, qw);
    // tf2::Quaternion q_tf;
    // q_tf.setRPY(0, 0, M_PI);  // world → footprint 的旋转，不知道为啥改为四元数不用修正了
    // tf2::Quaternion q_fp = q_tf.inverse() * q_input;
    // target_pose.orientation = tf2::toMsg(q_fp);
    // 构造输入 Pose（world 坐标系下）
    geometry_msgs::msg::Pose target_pose_world;
    target_pose_world.position.x = x;
    target_pose_world.position.y = y;
    target_pose_world.position.z = z;
    target_pose_world.orientation.x = qx;
    target_pose_world.orientation.y = qy;
    target_pose_world.orientation.z = qz;
    target_pose_world.orientation.w = qw;

    // ====== 坐标系变换：world → kuka_footprint ======

    // 1. 平移变换（R^T * (p - T)）
    Eigen::Vector3d T(1.0, 0.0, 0.0);
    Eigen::Matrix3d R;
    R << -1, 0, 0,
        0, -1, 0,
        0,  0, 1;

    Eigen::Vector3d p_world(x, y, z);
    Eigen::Vector3d p_fp = R.transpose() * (p_world - T);  // 坐标变换

    // 2. 姿态变换：q_fp = q_tf⁻¹ * q_input
    tf2::Quaternion q_input(qx, qy, qz, qw);
    q_input.normalize();  

    tf2::Quaternion q_tf;
    q_tf.setRPY(0, 0, M_PI);  // world → footprint 的旋转
    q_tf.normalize();

    tf2::Quaternion q_fp = q_tf.inverse() * q_input;
    q_fp.normalize();

    // ====== 赋值目标 Pose（footprint 坐标系下） ======
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = p_fp(0);
    target_pose.position.y = p_fp(1);
    target_pose.position.z = p_fp(2);
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

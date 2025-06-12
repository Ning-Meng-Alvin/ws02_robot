cmake_minimum_required(VERSION 3.5)
project(kuka_communication_server)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kuka_interfaces REQUIRED)

# 添加可执行文件
add_executable(pose_check_action_server src/pose_check_action_server.cpp)

# 链接依赖库
ament_target_dependencies(
  pose_check_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  kuka_interfaces
)

# 安装节点
install(TARGETS
  pose_check_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

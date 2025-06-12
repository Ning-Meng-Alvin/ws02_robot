cmake_minimum_required(VERSION 3.5)
project(kuka_communication_server)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kuka_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 添加可执行文件
add_executable(pose_check_action_server src/pose_check_action_server.cpp)
add_executable(cartesian_follow_action_server src/cartesian_follow_action_server.cpp)

# 设置依赖
ament_target_dependencies(pose_check_action_server
  rclcpp rclcpp_action moveit_ros_planning_interface
  geometry_msgs tf2_geometry_msgs Eigen3 kuka_interfaces
)

ament_target_dependencies(cartesian_follow_action_server
  rclcpp rclcpp_action moveit_ros_planning_interface
  geometry_msgs tf2_geometry_msgs Eigen3 kuka_interfaces
)

# 链接 Boost 库
target_link_libraries(pose_check_action_server Boost::filesystem Boost::system)
target_link_libraries(cartesian_follow_action_server Boost::filesystem Boost::system)

# 安装
install(TARGETS
  pose_check_action_server
  cartesian_follow_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(collision_object_manager_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(geometric_shapes REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 添加可执行文件
add_executable(collision_object_node src/collision_object_manager_node.cpp)

# 链接依赖库
ament_target_dependencies(
  collision_object_node
  rclcpp
  control_msgs
  moveit_ros_planning_interface
  Boost
  geometric_shapes
  ament_index_cpp
)


# 安装可执行文件
install(TARGETS
  collision_object_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()

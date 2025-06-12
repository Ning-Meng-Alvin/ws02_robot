# cmake_minimum_required(VERSION 3.8)
# project(demo_kuka_controller)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(Boost REQUIRED COMPONENTS filesystem)
# find_package(ament_index_cpp REQUIRED)

# include_directories(
#   include
# )

# add_executable(kuka_moveit_control
#   src/kuka_moveit_control.cpp
# )

# ament_target_dependencies(
#   kuka_moveit_control
#   rclcpp
#   moveit_ros_planning_interface
#   Boost
#   ament_index_cpp
# )


# install(
#   TARGETS kuka_moveit_control
#   DESTINATION lib/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

cmake_minimum_required(VERSION 3.8)
project(demo_kuka_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 和 MoveIt!
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 查找 Boost，并包含 filesystem 与 system 组件
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 包含头文件目录
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# 声明可执行文件
add_executable(kuka_moveit_control
  src/kuka_moveit_control.cpp
)

# 链接 ament（ROS）包依赖
ament_target_dependencies(
  kuka_moveit_control
  rclcpp
  moveit_ros_planning_interface
  ament_index_cpp
)

# 单独链接 Boost 库
target_link_libraries(kuka_moveit_control
  Boost::filesystem
  Boost::system
)

# 安装可执行文件
install(
  TARGETS kuka_moveit_control
  DESTINATION lib/${PROJECT_NAME}
)

# 测试相关（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


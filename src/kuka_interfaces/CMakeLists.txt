# cmake_minimum_required(VERSION 3.5)
# project(kuka_interfaces)

# # 1. 查找 ament 和生成接口的包
# find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# # 2. 查找所有用到的消息类型包
# find_package(geometry_msgs REQUIRED)
# find_package(action_msgs REQUIRED)

# # 3. 生成 srv 和 action 接口，同时声明依赖的消息包
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/CheckPose.action"
#   "action/CartesianFollow.action"
#   DEPENDENCIES
#     geometry_msgs
#     action_msgs
# )

# # 4. 导出运行时依赖，供下游包使用
# ament_export_dependencies(rosidl_default_runtime)

# # 5. 注册本包
# ament_package()

cmake_minimum_required(VERSION 3.5)
project(kuka_interfaces)

# 构建工具
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 消息依赖
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# 接口生成
set(MSG_DEPENDENCIES
  geometry_msgs
  action_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CheckPose.action"
  "action/CartesianFollow.action"
  DEPENDENCIES ${MSG_DEPENDENCIES}
)

# 导出接口包依赖
ament_export_dependencies(rosidl_default_runtime)
ament_package()


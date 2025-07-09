// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometric_shapes/shape_operations.h>
// #include <shape_msgs/msg/mesh.hpp>
// #include <fstream> 

// #include <geometric_shapes/mesh_operations.h>
// #include <geometric_shapes/shape_operations.h>
// #include <ament_index_cpp/get_package_share_directory.hpp>

// #include <cmath>

// // 定义一个将角度转换为弧度的函数
// double degreesToRadians(double degrees) {
//   return degrees * M_PI / 180.0;
// }

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("add_car");

//   // 创建 MoveGroupInterface 对象，指定规划组名称
//   moveit::planning_interface::MoveGroupInterface move_group(node, "kr210_arm");

//   // 创建 PlanningSceneInterface 对象
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//   // 添加圆柱体到规划场景
//   moveit_msgs::msg::CollisionObject collision_object;
//   collision_object.header.frame_id = move_group.getPlanningFrame();
//   collision_object.id = "cylinder";

//   shape_msgs::msg::SolidPrimitive primitive;
//   primitive.type = primitive.CYLINDER;
//   primitive.dimensions.resize(2);
//   primitive.dimensions[0] = 1;  // 高度
//   primitive.dimensions[1] = 0.5;  // 半径

// // xy要取相反数
//   geometry_msgs::msg::Pose cylinder_pose;
//   cylinder_pose.orientation.w = 1.0;
//   cylinder_pose.position.x = 1.0;
//   cylinder_pose.position.y = 1.0;
//   cylinder_pose.position.z = 0.5;

//   collision_object.primitives.push_back(primitive);
//   collision_object.primitive_poses.push_back(cylinder_pose);
//   collision_object.operation = collision_object.ADD;

//   std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object);

//   planning_scene_interface.applyCollisionObjects(collision_objects);

//   rclcpp::shutdown();
//   return 0;
// }



// #include <rclcpp/rclcpp.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/move_group_interface/move_group_interface.h>

// #include <geometric_shapes/mesh_operations.h>
// #include <geometric_shapes/shape_operations.h>
// #include <shape_msgs/msg/mesh.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>
// #include <geometry_msgs/msg/pose.hpp>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <limits>
// #include <fstream>
// #include <cmath>

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("add_car");

//   // 声明并读取参数：缩放、全局平移和旋转
//   node->declare_parameter<double>("scale", 0.031);
//   node->declare_parameter<double>("xt",    3.0);
//   node->declare_parameter<double>("yt",    0.0);
//   node->declare_parameter<double>("roll",  0.0);
//   node->declare_parameter<double>("pitch", 0.0);
//   node->declare_parameter<double>("yaw",  -M_PI/2.0);

//   double scale, xt, yt, roll, pitch, yaw;
//   node->get_parameter("scale", scale);
//   node->get_parameter("xt",    xt);
//   node->get_parameter("yt",    yt);
//   node->get_parameter("roll",  roll);
//   node->get_parameter("pitch", pitch);
//   node->get_parameter("yaw",   yaw);

//   // 初始化 MoveIt 接口
//   auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "kr210_arm");
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//   // 定位 STL 文件
//   std::string pkg       = ament_index_cpp::get_package_share_directory("collision_object_manager_cpp");
//   std::string mesh_file = pkg + "/meshes/Mini_Cooper_Simplified.stl";
//   if (!std::ifstream(mesh_file).good()) {
//     RCLCPP_ERROR(node->get_logger(), "STL 文件未找到：%s", mesh_file.c_str());
//     return 1;
//   }

//   // 加载并缩放原始网格（mm->m 或自定义）
//   shapes::Mesh* raw = shapes::createMeshFromResource("file://" + mesh_file);
//   if (!raw) {
//     RCLCPP_ERROR(node->get_logger(), "STL 加载失败！");
//     return 1;
//   }
//   for (size_t i = 0; i < raw->vertex_count; ++i) {
//     raw->vertices[3*i+0] *= scale;
//     raw->vertices[3*i+1] *= scale;
//     raw->vertices[3*i+2] *= scale;
//   }

//   // 计算包围盒：xmin,xmax,ymin,ymax,zmin
//   double xmin =  std::numeric_limits<double>::infinity(), xmax = -xmin;
//   double ymin =  std::numeric_limits<double>::infinity(), ymax = -ymin;
//   double zmin =  std::numeric_limits<double>::infinity();
//   for (size_t i = 0; i < raw->vertex_count; ++i) {
//     double x = raw->vertices[3*i+0];
//     double y = raw->vertices[3*i+1];
//     double z = raw->vertices[3*i+2];
//     xmin = std::min(xmin, x);  xmax = std::max(xmax, x);
//     ymin = std::min(ymin, y);  ymax = std::max(ymax, y);
//     zmin = std::min(zmin, z);
//   }
//   // 模型几何中心
//   double xc = 0.5 * (xmin + xmax);
//   double yc = 0.5 * (ymin + ymax);

//   // 在本地坐标系中将顶点“中心化并贴地”
//   for (size_t i = 0; i < raw->vertex_count; ++i) {
//     raw->vertices[3*i+0] -= xc;
//     raw->vertices[3*i+1] -= yc;
//     raw->vertices[3*i+2] -= zmin;
//   }

//   // 转成 ROS Mesh 消息
//   shapes::ShapeConstPtr shp(raw);
//   shapes::ShapeMsg   smsg;
//   shapes::constructMsgFromShape(shp.get(), smsg);
//   auto mesh_msg = boost::get<shape_msgs::msg::Mesh>(smsg);

//   // 构造世界坐标下的 Pose（先旋转再平移）
//   tf2::Quaternion q;
//   q.setRPY(roll, pitch, yaw);
//   geometry_msgs::msg::Pose pose;
//   pose.orientation = tf2::toMsg(q);
//   pose.position.x  = xt;      // 全局 XY 平移参数
//   pose.position.y  = yt;
//   pose.position.z  = 0.0;     // 已贴地

//   // 删除旧的，并发布新的碰撞体
//   const std::string object_id = "mini_cooper_collision_stl";
//   planning_scene_interface.removeCollisionObjects({object_id});

//   moveit_msgs::msg::CollisionObject co;
//   co.id              = object_id;
//   co.header.frame_id = move_group->getPlanningFrame();
//   co.meshes.push_back(mesh_msg);
//   co.mesh_poses.push_back(pose);
//   co.operation       = co.ADD;
//   planning_scene_interface.applyCollisionObjects({co});

//   RCLCPP_INFO(node->get_logger(),
//     "发布 [%s]：scale=%.3f, xt=%.2f, yt=%.2f, yaw=%.2f",
//     object_id.c_str(), scale, xt, yt, yaw);

//   // 保持运行，确保发布成功
//   rclcpp::spin_some(node);
//   rclcpp::shutdown();
//   return 0;
// }





// #include <rclcpp/rclcpp.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/move_group_interface/move_group_interface.h>

// #include <geometric_shapes/mesh_operations.h>
// #include <geometric_shapes/shape_operations.h>
// #include <shape_msgs/msg/mesh.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>
// #include <geometry_msgs/msg/pose.hpp>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include "kuka_interfaces/srv/update_collision_object_pose.hpp" 


// #include <limits>
// #include <fstream>
// #include <cmath>

// class CollisionObjectManager : public rclcpp::Node {
// public:
//   CollisionObjectManager() : Node("collision_object_service_node") {
//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "kr210_arm");
//     srv_ = this->create_service<kuka_interfaces::srv::UpdateCollisionObjectPose>(
//       "update_collision_object_pose",
//       std::bind(&CollisionObjectManager::updateCallback, this, std::placeholders::_1, std::placeholders::_2));
//   }

// private:
//   void updateCallback(const std::shared_ptr<kuka_interfaces::srv::UpdateCollisionObjectPose::Request> req,
//                       std::shared_ptr<kuka_interfaces::srv::UpdateCollisionObjectPose::Response> res) {
//     std::string pkg = ament_index_cpp::get_package_share_directory("collision_object_manager_cpp");
//     std::string mesh_file = pkg + "/meshes/Mini_Cooper_Simplified.stl";

//     if (!std::ifstream(mesh_file).good()) {
//       RCLCPP_ERROR(this->get_logger(), "STL 文件未找到：%s", mesh_file.c_str());
//       res->success = false;
//       return;
//     }

//     shapes::Mesh* raw = shapes::createMeshFromResource("file://" + mesh_file);
//     if (!raw) {
//       RCLCPP_ERROR(this->get_logger(), "STL 加载失败！");
//       res->success = false;
//       return;
//     }

//     double scale = 0.031;
//     for (size_t i = 0; i < raw->vertex_count; ++i) {
//       raw->vertices[3*i+0] *= scale;
//       raw->vertices[3*i+1] *= scale;
//       raw->vertices[3*i+2] *= scale;
//     }

//     double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
//     double ymin = std::numeric_limits<double>::infinity(), ymax = -ymin;
//     double zmin = std::numeric_limits<double>::infinity();
//     for (size_t i = 0; i < raw->vertex_count; ++i) {
//       double x = raw->vertices[3*i+0];
//       double y = raw->vertices[3*i+1];
//       double z = raw->vertices[3*i+2];
//       xmin = std::min(xmin, x);  xmax = std::max(xmax, x);
//       ymin = std::min(ymin, y);  ymax = std::max(ymax, y);
//       zmin = std::min(zmin, z);
//     }

//     double xc = 0.5 * (xmin + xmax);
//     double yc = 0.5 * (ymin + ymax);
//     for (size_t i = 0; i < raw->vertex_count; ++i) {
//       raw->vertices[3*i+0] -= xc;
//       raw->vertices[3*i+1] -= yc;
//       raw->vertices[3*i+2] -= zmin;
//     }

//     shapes::ShapeConstPtr shp(raw);
//     shapes::ShapeMsg smsg;
//     shapes::constructMsgFromShape(shp.get(), smsg);
//     auto mesh_msg = boost::get<shape_msgs::msg::Mesh>(smsg);

//     tf2::Quaternion q(req->qx, req->qy, req->qz, req->qw);
//     geometry_msgs::msg::Pose pose;
//     pose.orientation = tf2::toMsg(q);
//     pose.position.x = req->x;
//     pose.position.y = req->y;
//     pose.position.z = req->z;

//     moveit::planning_interface::PlanningSceneInterface psi;
//     moveit_msgs::msg::CollisionObject co;
//     co.id = req->collision_object_name;
//     co.header.frame_id = move_group_->getPlanningFrame();
//     co.meshes.push_back(mesh_msg);
//     co.mesh_poses.push_back(pose);
//     co.operation = co.ADD;

//     psi.removeCollisionObjects({req->collision_object_name});
//     psi.applyCollisionObjects({co});

//     RCLCPP_INFO(this->get_logger(), "更新 [%s] 位姿成功", req->collision_object_name.c_str());
//     res->success = true;
//   }

//   rclcpp::Service<kuka_interfaces::srv::UpdateCollisionObjectPose>::SharedPtr srv_;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CollisionObjectManager>());
//   rclcpp::shutdown();
//   return 0;
// }






// #include <rclcpp/rclcpp.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>

// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/move_group_interface/move_group_interface.h>

// #include <geometric_shapes/mesh_operations.h>
// #include <geometric_shapes/shape_operations.h>
// #include <shape_msgs/msg/mesh.hpp>
// #include <moveit_msgs/msg/collision_object.hpp>
// #include <geometry_msgs/msg/pose.hpp>

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include "kuka_interfaces/srv/update_collision_object_pose.hpp"

// #include <limits>
// #include <fstream>
// #include <cmath>

// class CollisionObjectManager : public rclcpp::Node {
// public:
//   CollisionObjectManager() : Node("collision_object_service_node") {
//     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this, "kr210_arm");
//     srv_ = this->create_service<kuka_interfaces::srv::UpdateCollisionObjectPose>(
//       "update_collision_object_pose",
//       std::bind(&CollisionObjectManager::updateCallback, this, std::placeholders::_1, std::placeholders::_2));
//   }

// private:
//   void updateCallback(const std::shared_ptr<kuka_interfaces::srv::UpdateCollisionObjectPose::Request> req,
//                       std::shared_ptr<kuka_interfaces::srv::UpdateCollisionObjectPose::Response> res) {
//     moveit::planning_interface::PlanningSceneInterface psi;
    
//     if (req->delete_only) {
//       psi.removeCollisionObjects({req->collision_object_name});
//       RCLCPP_INFO(this->get_logger(), "删除碰撞体 [%s]", req->collision_object_name.c_str());
//       res->success = true;
//       return;
//     }

//     moveit_msgs::msg::CollisionObject co;
//     co.id = req->collision_object_name;
//     co.header.frame_id = move_group_->getPlanningFrame();

//     geometry_msgs::msg::Pose pose;
//     tf2::Quaternion q(req->qx, req->qy, req->qz, req->qw);
//     pose.orientation = tf2::toMsg(q);
//     pose.position.x = req->x;
//     pose.position.y = req->y;
//     pose.position.z = req->z;

//     if (req->shape_type == "mesh") {
//       std::string pkg = ament_index_cpp::get_package_share_directory("collision_object_manager_cpp");
//       std::string mesh_file = pkg + "/meshes/" + req->mesh_name;

//       if (!std::ifstream(mesh_file).good()) {
//         RCLCPP_ERROR(this->get_logger(), "mesh 文件未找到: %s", mesh_file.c_str());
//         res->success = false;
//         return;
//       }

//       shapes::Mesh* raw = shapes::createMeshFromResource("file://" + mesh_file);
//       if (!raw) {
//         RCLCPP_ERROR(this->get_logger(), "mesh 加载失败!");
//         res->success = false;
//         return;
//       }

//       shapes::ShapeConstPtr shp(raw);
//       shapes::ShapeMsg smsg;
//       shapes::constructMsgFromShape(shp.get(), smsg);
//       auto mesh_msg = boost::get<shape_msgs::msg::Mesh>(smsg);
//       co.meshes.push_back(mesh_msg);
//       co.mesh_poses.push_back(pose);
//     } else if (req->shape_type == "box") {
//       shape_msgs::msg::SolidPrimitive box;
//       box.type = shape_msgs::msg::SolidPrimitive::BOX;
//       box.dimensions = {req->dim_x, req->dim_y, req->dim_z};
//       co.primitives.push_back(box);
//       co.primitive_poses.push_back(pose);
//     } else if (req->shape_type == "cylinder") {
//       shape_msgs::msg::SolidPrimitive cyl;
//       cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
//       cyl.dimensions = {req->dim_x, req->dim_y}; // height, radius
//       co.primitives.push_back(cyl);
//       co.primitive_poses.push_back(pose);
//     } else if (req->shape_type == "sphere") {
//       shape_msgs::msg::SolidPrimitive sph;
//       sph.type = shape_msgs::msg::SolidPrimitive::SPHERE;
//       sph.dimensions = {req->dim_x}; // radius
//       co.primitives.push_back(sph);
//       co.primitive_poses.push_back(pose);
//     } else {
//       RCLCPP_ERROR(this->get_logger(), "shape_type 无效: %s", req->shape_type.c_str());
//       res->success = false;
//       return;
//     }

//     co.operation = co.ADD;
//     psi.removeCollisionObjects({req->collision_object_name});
//     psi.applyCollisionObjects({co});

//     RCLCPP_INFO(this->get_logger(), "更新 [%s] 碰撞体成功", req->collision_object_name.c_str());
//     res->success = true;
//   }

//   rclcpp::Service<kuka_interfaces::srv::UpdateCollisionObjectPose>::SharedPtr srv_;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CollisionObjectManager>());
//   rclcpp::shutdown();
//   return 0;
// } 指针问题




#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <limits>
#include <fstream>
#include <cmath>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_car");

  // 声明并读取参数
  node->declare_parameter<double>("scale", 0.031);
  node->declare_parameter<double>("xt",    3.0);
  node->declare_parameter<double>("yt",    0.0);
  node->declare_parameter<double>("roll",  0.0);
  node->declare_parameter<double>("pitch", 0.0);
  node->declare_parameter<double>("yaw",  -M_PI / 2.0);

  double scale, xt, yt, roll, pitch, yaw;
  node->get_parameter("scale", scale);
  node->get_parameter("xt",    xt);
  node->get_parameter("yt",    yt);
  node->get_parameter("roll",  roll);
  node->get_parameter("pitch", pitch);
  node->get_parameter("yaw",   yaw);

  // 初始化 MoveIt 接口
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "kr210_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 定位 STL 文件
  std::string pkg = ament_index_cpp::get_package_share_directory("collision_object_manager_cpp");
  std::string mesh_file = pkg + "/meshes/Mini_Cooper_Simplified.stl";
  if (!std::ifstream(mesh_file).good()) {
    RCLCPP_ERROR(node->get_logger(), "STL 文件未找到：%s", mesh_file.c_str());
    return 1;
  }

  // 加载并缩放网格
  shapes::Mesh* raw = shapes::createMeshFromResource("file://" + mesh_file);
  if (!raw) {
    RCLCPP_ERROR(node->get_logger(), "STL 加载失败！");
    return 1;
  }

  for (size_t i = 0; i < raw->vertex_count; ++i) {
    raw->vertices[3 * i + 0] *= scale;
    raw->vertices[3 * i + 1] *= scale;
    raw->vertices[3 * i + 2] *= scale;
  }

  // 计算包围盒中心和最低点
  double xmin =  std::numeric_limits<double>::infinity(), xmax = -xmin;
  double ymin =  std::numeric_limits<double>::infinity(), ymax = -ymin;
  double zmin =  std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < raw->vertex_count; ++i) {
    double x = raw->vertices[3 * i + 0];
    double y = raw->vertices[3 * i + 1];
    double z = raw->vertices[3 * i + 2];
    xmin = std::min(xmin, x); xmax = std::max(xmax, x);
    ymin = std::min(ymin, y); ymax = std::max(ymax, y);
    zmin = std::min(zmin, z);
  }

  double xc = 0.5 * (xmin + xmax);
  double yc = 0.5 * (ymin + ymax);

  // 平移使模型中心贴地
  for (size_t i = 0; i < raw->vertex_count; ++i) {
    raw->vertices[3 * i + 0] -= xc;
    raw->vertices[3 * i + 1] -= yc;
    raw->vertices[3 * i + 2] -= zmin;
  }

  // 转为 ROS 消息格式
  shapes::ShapeConstPtr shp(raw);
  shapes::ShapeMsg smsg;
  shapes::constructMsgFromShape(shp.get(), smsg);
  auto mesh_msg = boost::get<shape_msgs::msg::Mesh>(smsg);

  // 构造全局姿态 Pose
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Pose pose;
  pose.orientation = tf2::toMsg(q);
  pose.position.x = xt;
  pose.position.y = yt;
  pose.position.z = 0.0;

  // 删除原对象，添加新碰撞体
  const std::string object_id = "mini_cooper_collision_stl";
  planning_scene_interface.removeCollisionObjects({object_id});

  moveit_msgs::msg::CollisionObject co;
  co.id = object_id;
  co.header.frame_id = move_group->getPlanningFrame();
  co.meshes.push_back(mesh_msg);
  co.mesh_poses.push_back(pose);
  co.operation = co.ADD;

  planning_scene_interface.applyCollisionObjects({co});

  RCLCPP_INFO(node->get_logger(),
              "发布 [%s]：scale=%.3f, xt=%.2f, yt=%.2f, yaw=%.2f",
              object_id.c_str(), scale, xt, yt, yaw);

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}

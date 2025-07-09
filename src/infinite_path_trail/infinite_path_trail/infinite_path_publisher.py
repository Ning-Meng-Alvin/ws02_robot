# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time
# from rclpy.duration import Duration
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped, Point, Quaternion
# import tf2_ros
# from tf2_ros import LookupException, ExtrapolationException, ConnectivityException

# class Tool0PathPublisher(Node):
#     def __init__(self):
#         super().__init__('tool0_path_publisher')

#         # TF 缓冲区和监听器
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Path 发布器
#         self.pub = self.create_publisher(Path, 'tool0_path', 10)

#         # 初始化 Path 消息
#         self.path = Path()
#         self.path.header.frame_id = 'world'  # ← 根据你 TF 树设置

#         # 定时器：5Hz
#         self.create_timer(0.2, self.timer_callback)

#     def timer_callback(self):
#         # 等待 TF 可用
#         try:
#             ready = self.tf_buffer.can_transform(
#                 'world', 'tool0', Time(), timeout=Duration(seconds=1.0)
#             )
#         except Exception as e:
#             self.get_logger().warn(f'can_transform 检查失败: {e}')
#             return

#         if not ready:
#             self.get_logger().warn('等待 TF world → tool0 中...')
#             return

#         try:
#             # 查询 TF 变换
#             trans = self.tf_buffer.lookup_transform('world', 'tool0', Time())

#             # 构造路径点
#             pose = PoseStamped()
#             pose.header = trans.header
#             pose.pose.position = Point(
#                 x=trans.transform.translation.x,
#                 y=trans.transform.translation.y,
#                 z=trans.transform.translation.z
#             )
#             pose.pose.orientation = Quaternion(
#                 x=trans.transform.rotation.x,
#                 y=trans.transform.rotation.y,
#                 z=trans.transform.rotation.z,
#                 w=trans.transform.rotation.w
#             )

#             # 添加进 Path 并发布
#             self.path.header.stamp = self.get_clock().now().to_msg()
#             self.path.poses.append(pose)

#             # 可选：限制点数上限
#             if len(self.path.poses) > 8000:
#                 self.path.poses = self.path.poses[-8000:]

#             self.pub.publish(self.path)

#         except (LookupException, ExtrapolationException, ConnectivityException) as e:
#             self.get_logger().warn(f'TF 查询失败: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = Tool0PathPublisher()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import tf2_ros
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException

class Tool0PathPublisher(Node):
    def __init__(self):
        super().__init__('tool0_path_publisher')

        # TF 缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Path 发布器
        self.pub = self.create_publisher(Path, 'tool0_path', 10)

        # 初始化 Path 消息
        self.path = Path()
        self.path.header.frame_id = 'world'  # ← 根据你 TF 树设置

        # 定时器：5Hz
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        try:
            # 检查 TF 是否可用
            ready = self.tf_buffer.can_transform(
                'world', 'tool0', Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f'can_transform 检查失败: {e}')
            return

        if not ready:
            self.get_logger().warn('等待 TF world → tool0 中...')
            return

        try:
            # 查询 TF
            trans = self.tf_buffer.lookup_transform('world', 'tool0', Time())

            # 构造路径点
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position = Point(
                x=trans.transform.translation.x,
                y=trans.transform.translation.y,
                z=trans.transform.translation.z
            )
            pose.pose.orientation = Quaternion(
                x=trans.transform.rotation.x,
                y=trans.transform.rotation.y,
                z=trans.transform.rotation.z,
                w=trans.transform.rotation.w
            )

            # 添加并发布 Path
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path.poses.append(pose)

            if len(self.path.poses) > 8000:
                self.path.poses = self.path.poses[-8000:]

            self.pub.publish(self.path)

            # 打印当前位姿
            self.get_logger().info(
                f"tool0位置: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f} | "
                f"姿态: x={pose.pose.orientation.x:.3f}, y={pose.pose.orientation.y:.3f}, "
                f"z={pose.pose.orientation.z:.3f}, w={pose.pose.orientation.w:.3f}"
            )

        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            self.get_logger().warn(f'TF 查询失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Tool0PathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

# import rclpy
# from rclpy.node import Node
# from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# from tf2_ros.transform_broadcaster import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# from scipy.spatial.transform import Rotation as R
# import math

# class TFPublisher(Node):
#     def __init__(self):
#         super().__init__('kr210_tf_initializer')

#         self.static_broadcaster = StaticTransformBroadcaster(self)
#         self.dynamic_broadcaster = TransformBroadcaster(self)

#         # 静态 TF：world → kr210_footprint，Z轴旋转180°
#         self.publish_static_transform(
#             parent_frame="world",
#             child_frame="kuka_footprint",
#             translation=[1.0, 0.0, 0.0],
#             rpy=[0.0, 0.0, math.pi]
#             # rpy=[0.0, 0.0, 0.0]
#         )

#         # 定时发布 kr210_link6 → camera_position
#         self.timer = self.create_timer(0.1, self.publish_camera_tf)

#     def rpy_to_quat(self, rpy):
#         """将 RPY 欧拉角转换为四元数 [x, y, z, w]"""
#         quat = R.from_euler('xyz', rpy).as_quat()
#         return quat  # 注意顺序为 [x, y, z, w]

#     def publish_static_transform(self, parent_frame, child_frame, translation, rpy):
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = parent_frame
#         t.child_frame_id = child_frame
#         t.transform.translation.x = translation[0]
#         t.transform.translation.y = translation[1]
#         t.transform.translation.z = translation[2]

#         quat = self.rpy_to_quat(rpy)
#         t.transform.rotation.x = quat[0]
#         t.transform.rotation.y = quat[1]
#         t.transform.rotation.z = quat[2]
#         t.transform.rotation.w = quat[3]

#         self.static_broadcaster.sendTransform(t)

#     def publish_camera_tf(self):
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = "kr210_link6"
#         t.child_frame_id = "camera_position"
#         t.transform.translation.x = 0.1
#         t.transform.translation.y = 0.0
#         t.transform.translation.z = -0.2

#         quat = self.rpy_to_quat([(math.pi)/2 ,0.0,(math.pi)/2])
#         t.transform.rotation.x = quat[0]
#         t.transform.rotation.y = quat[1]
#         t.transform.rotation.z = quat[2]
#         t.transform.rotation.w = quat[3]
   

#         self.dynamic_broadcaster.sendTransform(t)

# def main():
#     rclpy.init()
#     node = TFPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import math

class TFPublisher(Node):
    def __init__(self):
        super().__init__('kuka__tf_initializer')
        self.broadcaster = TransformBroadcaster(self)
        # 0.1s 发布一次（10Hz）
        self.timer = self.create_timer(0.1, self.publish_footprint_tf)

    def rpy_to_quat(self, rpy):
        return R.from_euler('xyz', rpy).as_quat()

    def publish_footprint_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'kuka_footprint'
        # 平移
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # 旋转 180°
        q = self.rpy_to_quat([0.0, 0.0, math.pi])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

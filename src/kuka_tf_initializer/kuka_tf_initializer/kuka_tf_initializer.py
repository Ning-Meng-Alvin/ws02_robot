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

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import rclpy.time
import time


class TFRepublisher(Node):
    def __init__(self):
        super().__init__('robot_tf_broadcaster_node')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        #  kr210 link + camera_position
        self.target_links = [
            'kr210_base_link', 'kr210_link1', 'kr210_link2',
            'kr210_link3', 'kr210_link4', 'kr210_link5',
            'kr210_link6', 'camera_position'
        ]

        self.get_logger().info("Waiting 2 seconds for TF buffer to fill...")
        time.sleep(2.0)

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        now = rclpy.time.Time()
        for link in self.target_links:
            try:
                if self.tf_buffer.can_transform('world', link, now, timeout=rclpy.duration.Duration(seconds=0.2)):
                    t = self.tf_buffer.lookup_transform('world', link, now)

                    # 更新时间戳
                    t.header.stamp = self.get_clock().now().to_msg()
                    # 防重复名
                    t.child_frame_id = f"{link}_world"

                    self.broadcaster.sendTransform(t)

                    if link == 'camera_position':
                        trans = t.transform.translation
                        rot = t.transform.rotation
                        self.get_logger().info(
                            f"[{link}] pos: ({trans.x:.3f}, {trans.y:.3f}, {trans.z:.3f}) | "
                            f"quat: ({rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f})"
                        )
                else:
                    self.get_logger().warn(f"No TF available yet: world → {link}")
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed for {link}: {e}")


def main():
    rclpy.init()
    node = TFRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

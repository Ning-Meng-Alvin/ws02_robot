import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kuka_interfaces.action import CartesianFollow
from geometry_msgs.msg import Pose

class CartesianFollowClient(Node):
    def __init__(self):
        super().__init__('cartesian_follow_action_client')
        self._client = ActionClient(self, CartesianFollow, 'cartesian_follow')

    def send_goal(self):
        self._client.wait_for_server()

        # 构造 L 形轨迹点（前 5 个 x 方向，后 5 个 z 方向）
        poses = []
        fixed_orientation = [0.0, 0.0, 0.0, 1.0]
        start_x, start_y, start_z = 0.5, 0.0, 0.5

        for i in range(5):  # 横线：x 增长
            p = Pose()
            p.position.x = start_x + i * 0.02
            p.position.y = start_y
            p.position.z = start_z
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = fixed_orientation
            poses.append(p)

        for i in range(5):  # 竖线：z 增长
            p = Pose()
            p.position.x = start_x + 0.08
            p.position.y = start_y
            p.position.z = start_z + i * 0.02
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = fixed_orientation
            poses.append(p)

        goal_msg = CartesianFollow.Goal()
        goal_msg.path = poses
        goal_msg.end_effector = "tool0"

        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('❌ 目标被拒绝')
            return
        self.get_logger().info('✅ 目标已接受，开始执行...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        progress = feedback_msg.feedback.progress
        self.get_logger().info(f'📈 路径执行进度: {progress * 100:.1f}%')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'🎉 成功执行: {result.message}')
        else:
            self.get_logger().info(f'⚠️ 失败: {result.message}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CartesianFollowClient()
    node.send_goal()
    rclpy.spin(node)

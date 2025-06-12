import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kuka_interfaces.action import CartesianFollow
from geometry_msgs.msg import Pose
import random

class CartesianFollowClient(Node):
    def __init__(self):
        super().__init__('cartesian_follow_action_client')
        self._client = ActionClient(self, CartesianFollow, 'cartesian_follow')
    
    # def send_goal(self):
    #     self._client.wait_for_server()
       
    #     csv_path = '/home/liyq/ws02_robot/src/kuka_communication_client/csv/traj.csv'

    #     poses = []
    #     fixed_orientation = [0.0, 0.0, 0.7071, 0.7071]

    #     # 从 CSV 读取轨迹点
    #     with open(csv_path, newline='') as csvfile:
    #         reader = csv.DictReader(csvfile)
    #         for row in reader:
    #             p = Pose()
    #             p.position.x = float(row['x'])
    #             p.position.y = float(row['y'])
    #             p.position.z = float(row['z'])
    #             p.orientation.x = fixed_orientation[0]
    #             p.orientation.y = fixed_orientation[1]
    #             p.orientation.z = fixed_orientation[2]
    #             p.orientation.w = fixed_orientation[3]
    #             poses.append(p)

    #     # 构造目标消息
    #     goal_msg = CartesianFollow.Goal()
    #     goal_msg.path = poses
    #     goal_msg.end_effector = "tool0"

    #     self._send_goal_future = self._client.send_goal_async(
    #         goal_msg,
    #         feedback_callback=self.feedback_callback
    #     )
    #     self._send_goal_future.add_done_callback(self.goal_response_callback)



    def send_goal(self):
        self._client.wait_for_server()

        # 构造轨迹点
        poses = []
        start_x, start_y, start_z = 2.0, 1.2, 1.7
        fixed_orientation = [0.0, 0.0, 0.7071, 0.7071]
        for i in range(10):
            p = Pose()
            p.position.x = start_x + i * 0.05 + random.uniform(-0.01, 0.01)
            p.position.y = start_y - i * 0.05 + random.uniform(-0.01, 0.01)
            p.position.z = start_z - i * 0.05 + random.uniform(-0.01, 0.01)
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
            self.get_logger().info('目标被拒绝')
            return
        self.get_logger().info('目标已接受，开始执行...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        progress = feedback_msg.feedback.progress
        self.get_logger().info(f'路径执行进度: {progress * 100:.1f}%')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'成功执行: {result.message}')
        else:
            self.get_logger().info(f'失败: {result.message}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CartesianFollowClient()
    node.send_goal()
    rclpy.spin(node)

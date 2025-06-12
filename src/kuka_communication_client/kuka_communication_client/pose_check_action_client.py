# from rclpy.node import Node
# from rclpy.action import ActionClient
# from kuka_interfaces.action import CheckPose
# from geometry_msgs.msg import PoseStamped


# class PoseCheckClient(Node):
#     def __init__(self):
#         super().__init__('pose_check_action_client')
#         self._client = ActionClient(self, CheckPose, 'pose_check_action')
#         self._goals = []
#         self._results = []

#     def send_serial_goals(self, goals):
#         self._goals = goals
#         self._results = []

#         self.get_logger().info("客户端启动，等待服务端连接...")
#         self._client.wait_for_server()
#         self.get_logger().info("服务端已连接，开始发送目标位姿...")
#         self.send_next_goal()

#     def send_next_goal(self):
#         if not self._goals:
#             self.get_logger().info('所有目标检查完毕，结果如下：')
#             for i, res in enumerate(self._results):
#                 status = '可达' if res.success[0] else '不可达'
#                 self.get_logger().info(f'  目标 {i+1}: {status}')
#             # rclpy.shutdown()
#             return

#         goal_data = self._goals.pop(0)
#         goal_msg = CheckPose.Goal()

#         ps = PoseStamped()
#         ps.pose.position.x = goal_data['x']
#         ps.pose.position.y = goal_data['y']
#         ps.pose.position.z = goal_data['z']
#         ps.pose.orientation.x = goal_data['qx']
#         ps.pose.orientation.y = goal_data['qy']
#         ps.pose.orientation.z = goal_data['qz']
#         ps.pose.orientation.w = goal_data['qw']

#         goal_msg.poses = [ps]
#         goal_msg.end_effector = goal_data['end_effector']
#         goal_msg.execute = goal_data['execute']

#         self.get_logger().info(
#             f'发送目标 {len(self._results)+1}: x={ps.pose.position.x:.2f}, y={ps.pose.position.y:.2f}, z={ps.pose.position.z:.2f}, ee={goal_msg.end_effector}'
#         )

#         self._client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().warn('目标被拒绝')
#             self._results.append(CheckPose.Result(success=[False]))
#             self.send_next_goal()
#             return

#         self.get_logger().info('目标已接受，等待结果...')
#         goal_handle.get_result_async().add_done_callback(self.result_callback)

#     def result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'结果接收成功: success={result.success[0]}')
#         self._results.append(result)
#         self.send_next_goal()


# # 在文件底部加上这个函数
# def create_client():
#     return PoseCheckClient()

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kuka_interfaces.action import CheckPose
from geometry_msgs.msg import PoseStamped

class PoseCheckClient(Node):
    def __init__(self):
        super().__init__('pose_check_action_client')
        self._client = ActionClient(self, CheckPose, 'pose_check_action')
        self._goals = []
        self._results = []
        self._callback = None

    def send_serial_goals(self, goals, callback=None):
  
        self._goals = goals.copy()
        self._raw_goals = goals.copy() 
        self._results = []
        self._callback = callback

        self.get_logger().info("客户端启动，等待服务端连接...")
        self._client.wait_for_server()
        self.get_logger().info("服务端已连接，开始发送目标位姿...")
        self.send_next_goal()

    def send_next_goal(self):
        if not self._goals:
            self.get_logger().info('所有目标检查完毕，结果如下：')
            for i, res in enumerate(self._results):
                status = '可达' if res.success[0] else '不可达'
                self.get_logger().info(f'  目标 {i+1}: {status}')

            if self._callback:
                self._callback(self._results, self._raw_goals)
            return

        goal_data = self._goals.pop(0)
        goal_msg = CheckPose.Goal()

        ps = PoseStamped()
        ps.pose.position.x = goal_data['x']
        ps.pose.position.y = goal_data['y']
        ps.pose.position.z = goal_data['z']
        ps.pose.orientation.x = goal_data['qx']
        ps.pose.orientation.y = goal_data['qy']
        ps.pose.orientation.z = goal_data['qz']
        ps.pose.orientation.w = goal_data['qw']

        goal_msg.poses = [ps]
        goal_msg.end_effector = goal_data['end_effector']
        goal_msg.execute = goal_data['execute']

        self.get_logger().info(
            f'发送目标 {len(self._results)+1}: x={ps.pose.position.x:.2f}, y={ps.pose.position.y:.2f}, z={ps.pose.position.z:.2f}, ee={goal_msg.end_effector}'
        )

        self._client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('目标被拒绝')
            self._results.append(CheckPose.Result(success=[False]))
            self.send_next_goal()
            return

        self.get_logger().info('目标已接受，等待结果...')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'结果接收成功: success={result.success[0]}')
        self._results.append(result)
        self.send_next_goal()

def create_client():
    return PoseCheckClient()

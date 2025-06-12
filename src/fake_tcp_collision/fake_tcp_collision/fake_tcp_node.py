import rclpy
from rclpy.node import Node
from kuka_communication_client.pose_check_action_client import create_client

class FakeTCPNode(Node):
    def __init__(self):
        super().__init__('fake_tcp_node')
        self.get_logger().info("FakeTCP 启动")
        self.client_node = create_client()

        self.fake_tcp_data = [
            {'end_effector': 'camera_position', 'x': 0.3,   'y': 0.0, 'z': 1.5, 'qx': 0.183,  'qy': -0.683, 'qz': 0.183, 'qw': 0.683, 'score': 90,  'execute': False},
            {'end_effector': 'camera_position', 'x': 0.3,   'y': 0.0, 'z': 1.5, 'qx': -0.183, 'qy': -0.683, 'qz': 0.183, 'qw': 0.683, 'score': 99,  'execute': False},
            {'end_effector': 'camera_position', 'x': 999.0, 'y': 0.0, 'z': 1.5, 'qx': 0.183,  'qy': -0.683, 'qz': 0.183, 'qw': 0.683, 'score': 60,  'execute': False},
            {'end_effector': 'camera_position', 'x': 929.0, 'y': 0.0, 'z': 1.5, 'qx': 0.183,  'qy': -0.683, 'qz': 0.183, 'qw': 0.683, 'score': 120, 'execute': False},
            {'end_effector': 'camera_position', 'x': 0.3,   'y': 0.0, 'z': 1.7, 'qx': -0.183, 'qy': -0.683, 'qz': 0.183, 'qw': 0.683, 'score': 50,  'execute': False},
        ]

        # 启动任务，并传入回调
        self.client_node.send_serial_goals(self.fake_tcp_data, callback=self.handle_results)

    def handle_results(self, results, goals):
        reachable = []
        for r, g in zip(results, goals):
            if r.success[0]:
                reachable.append((g['score'], g))

        if not reachable:
            self.get_logger().info("没有可达的目标")
        else:
            best = max(reachable, key=lambda x: x[0])[1]
            self.get_logger().info(f"最优可达目标:score={best['score']}，位置=({best['x']:.2f}, {best['y']:.2f}, {best['z']:.2f})")



def main(args=None):
    rclpy.init(args=args)
    fake_tcp_node = FakeTCPNode()
    rclpy.spin(fake_tcp_node.client_node)
    fake_tcp_node.client_node.destroy_node()
    fake_tcp_node.destroy_node()
    rclpy.shutdown()

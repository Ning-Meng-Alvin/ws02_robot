import rclpy
from rclpy.node import Node

class CartesianFollowNode(Node):
    def __init__(self):
        super().__init__('cartesian_follow_node')
        self.get_logger().info("Cartesian follow node started")

def main(args=None):
    rclpy.init(args=args)
    node = CartesianFollowNode()
    rclpy.spin(node)
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
from kuka_interfaces.srv import UpdateCollisionObjectPose

class SceneManager(Node):
    def __init__(self):
        super().__init__('scene_manager_node')
        self.cli = self.create_client(UpdateCollisionObjectPose, 'update_collision_object_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务 update_collision_object_pose ...')

        self.load_vehicle()

    def load_vehicle(self):
        req = UpdateCollisionObjectPose.Request()
        req.collision_object_name = 'vehicle_1'
        req.shape_type = 'mesh'
        req.mesh_name = 'Mini_Cooper_Simplified.stl' 
        req.dim_x = 0.0
        req.dim_y = 0.0
        req.dim_z = 0.0
        req.x = 1.5
        req.y = 0.0
        req.z = 0.0
        req.qx = 0.0
        req.qy = 0.0
        req.qz = 0.707
        req.qw = 0.707
        req.delete_only = False

        future = self.cli.call_async(req)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('成功加载车辆模型！')
            else:
                self.get_logger().error('加载失败。')
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SceneManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

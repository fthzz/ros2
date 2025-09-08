import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer #坐标监听器
from tf_transformations import euler_from_quaternion #欧拉角转四元数
import math 

class TFBroadCaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.timer_ = self.create_timer(1.0, self.get_transform) 

    #查询坐标关系
    def get_transform(self):
        try:
            result = self.buffer_.lookup_transform('base_link', 'bottle_link',
                     rclpy.time.Time(seconds=0.0), rclpy.time.Duration(seconds=1.0))
            transform = result.transform
            self.get_logger().info(f'平移: {transform.translation}')
            self.get_logger().info(f'旋转: {transform.rotation}')
            self.get_logger().info(f'欧拉角: {euler_from_quaternion([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])}')
        except Exception as e:
            self.get_logger().info(f'Exception {e}')
            return

def main():
    rclpy.init()
    node = TFBroadCaster()
    rclpy.spin(node)
    rclpy.shutdown()

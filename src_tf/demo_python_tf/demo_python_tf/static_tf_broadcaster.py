import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster #静态坐标发布器
from geometry_msgs.msg import TransformStamped #消息接口
from tf_transformations import quaternion_from_euler #欧拉角转四元数
import math 

class StaticTFBroadCaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    #发布静态tf
    def publish_static_tf(self):
        #从base_link到camera_link的坐标关系
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link' #父坐标系名称
        transform.child_frame_id = 'camera_link' #子坐标系名称

        transform.transform.translation.x = 0.5
        transform.transform.translation.y = 0.3
        transform.transform.translation.z = 0.6

        #欧拉角转四元数 返回值为元组 xyzw
        q = quaternion_from_euler(math.radians(180), 0, 0) #math用于角度转弧度
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        #发布静态坐标变换
        self.static_broadcaster_.sendTransform(transform)
        self.get_logger().info(f'{transform}')
def main():
    rclpy.init()
    node = StaticTFBroadCaster()
    rclpy.spin(node)
    rclpy.shutdown()

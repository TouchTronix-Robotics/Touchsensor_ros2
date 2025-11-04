import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.sub = self.create_subscription(Image, '/sensor/matrix_sensor', self.cb, 10)

    def cb(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.int32)
        arr = arr.reshape(msg.height, msg.width)
        print(arr)
        self.get_logger().info(f'Got frame {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, mean={arr.mean()}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

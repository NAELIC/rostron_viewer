import rclpy
from PySide6.QtCore import QThread

from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class ROS2Thread(QThread):

    def run(self):
        print("ros 2")
        rclpy.init()

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

        minimal_publisher.destroy_node()
        rclpy.shutdown()
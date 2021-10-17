import rclpy
from PySide6.QtCore import QObject, QThread, Signal

from rclpy.node import Node

from rostron_interfaces.msg import Robots, Ball, Field
from rostron_utils.decorators import singleton


@singleton
class SignalHandler(QObject):
    field = Signal(Field)
    ball = Signal(Ball)

    allies = Signal(Robots)
    opponents = Signal(Robots)


class ROSTron_handler(Node):

    field: Field = None
    ball: Ball = None
    allies: Robots = None
    opponents: Robots = None

    def __init__(self):
        super().__init__('minimal_publisher')
        self.field_sub_ = self.create_subscription(
            Field, 'field', self.field_callback, 10)
        self.ball_sub_ = self.create_subscription(
            Ball, 'ball', self.ball_callback, 10)

    def field_callback(self, msg: Field):
        if not(self.field == msg):
            SignalHandler().field.emit(msg)
            self.field = msg

    def ball_callback(self, msg: Ball):
        if not(self.ball == msg):
            SignalHandler().ball.emit(msg)
            self.ball = msg


class ROS2Thread(QThread):
    def run(self):
        rclpy.init()
        node = ROSTron_handler()

        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

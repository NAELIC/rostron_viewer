import rclpy
from PySide6.QtCore import QObject, QThread, Signal

from rclpy.node import Node

from rostron_interfaces.msg import Robots, Ball, Field
from rostron_utils.decorators import singleton
from rcl_interfaces.srv import GetParameters

@singleton
class SignalHandler(QObject):
    field = Signal(Field)
    ball = Signal(Ball)

    allies = Signal(Robots)
    opponents = Signal(Robots)

    yellow = Signal(bool)

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

        # Yellow parameter
        self.param = self.create_client(GetParameters, 'vision/get_parameters')
        while not self.param.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        req = GetParameters.Request()
        req.names = ['yellow']
        future_req = self.param.call_async(request=req)
        rclpy.spin_until_future_complete(self, future=future_req)
        response = future_req.result()
        SignalHandler().yellow.emit(response.values[0].bool_value)

    def field_callback(self, msg: Field):
        if not(self.field == msg):
            self._logger.info(f"Qwidget {msg.goal_depth}")
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

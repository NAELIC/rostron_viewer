import PySide6
from PySide6.QtCore import QLine, QObject, Qt, QThread

from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from mainwindow import Ui_MainWindow
import sys

import rclpy
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


class Field(QWidget):
    def __init__(self,  parent=None) -> None:
        super().__init__(parent)
        self.show()
        self.ros_thread = ROS2Thread(parent=self)
        # self.connect(self.ros_thread, self.ros_thread.finished, self.ros_thread, QObject.deleteLater)
        self.ros_thread.start()

    def paintEvent(self, event: PySide6.QtGui.QPaintEvent) -> None:
        print("called")
        painter = QPainter(self)
        painter.setPen(QPen(Qt.red, 15))
        brush = QBrush(QColor(0, 0, 255, 255))
        painter.setBrush(brush)
        line = QLine(30, 30, 500, 30)
        painter.drawLine(line)


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.show()
        # self.field = Field(self)
        self.field = Field(self.field_container)
        vlayout = QVBoxLayout()
        vlayout.addWidget(self.field)
        self.field_container.setLayout(vlayout)


app = QApplication(sys.argv)
w = MainWindow()
app.exec()

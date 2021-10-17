import PySide6
from PySide6.QtCore import QLine, Qt

from .ros_handler import ROS2Thread, SignalHandler

from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from .mainwindow import Ui_MainWindow
import sys
from rostron_interfaces.msg import Field, Ball


class Field(QWidget):

    def field_callback(self, msg: Field):
        print(f"Qwidget {msg.goal_depth}")

    def ball_callback(self, msg: Ball):
        print(f"Qwidget ball {msg.position.x}")

    def __init__(self,  parent=None) -> None:
        super().__init__(parent)
        self.show()
        self.ros_thread = ROS2Thread(parent=self)
        self.ros_thread.start()
        SignalHandler().field.connect(self.field_callback)
        SignalHandler().ball.connect(self.ball_callback)

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

def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    app.exec()

import sys
import os

from PySide6.QtCore import QUrl
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from PySide6.QtWebChannel import QWebChannel
from PySide6.QtWebEngineWidgets import QWebEngineView

from rostron_interfaces.msg import Field, Ball
from .ros_handler import ROS2Thread, SignalHandler
from .backend import Backend

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

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle('ROSTron Viewer')

        # Field
        self.web = QWebEngineView(self)
        self.channel = QWebChannel()
        self.web.page().setWebChannel(self.channel)
        self.backend = Backend()
        self.channel.registerObject("backend", self.backend)

        url = QUrl.fromLocalFile(os.path.join(
            os.path.dirname(__file__), "index.html"))
        self.web.load(url)
        self.setCentralWidget(self.web)

        # ROS2 Thread
        self.ros_thread = ROS2Thread(parent=self)
        self.ros_thread.start()

        # Signal Handler

        SignalHandler().field.connect(self.backend.set_field)

        


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.showMaximized()
    app.exec()

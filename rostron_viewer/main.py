import sys
import os

from PySide6.QtCore import QUrl
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtWebChannel import QWebChannel
from PySide6.QtWebEngineWidgets import QWebEngineView

from .ros_handler import ROS2Thread, SignalHandler
from .backend import Backend

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

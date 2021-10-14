from typing import Optional
import PySide6
from PySide6.QtCore import QLine, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QApplication, QBoxLayout, QMainWindow, QVBoxLayout, QWidget
from mainwindow import Ui_MainWindow
import sys

class Field(QWidget):
    def __init__(self,  parent = None) -> None:
        super().__init__(parent)
        self.show()
    
    def paintEvent(self, event: PySide6.QtGui.QPaintEvent) -> None:
        print("called")
        painter = QPainter(self)
        painter.setPen(QPen(Qt.red, 15))
        brush = QBrush(QColor(0, 0, 255, 255))
        painter.setBrush(brush)
        line = QLine(30,30, 500, 30)
        painter.drawLine(line)

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setupUi(self)
        self.show()
        # self.field = Field(self)
        self.test = Field(self.field)
        # self.test.paintEvent()
        vlayout = QVBoxLayout()
        vlayout.addWidget(self.test)
        self.field.setLayout(vlayout)
        print(self.field.layout())
        
app = QApplication(sys.argv)
w = MainWindow()
app.exec()

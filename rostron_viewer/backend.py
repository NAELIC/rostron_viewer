from PySide6.QtCore import QObject
from PySide6.QtCore import Slot
from rostron_interfaces.msg import Field, Ball

from rosidl_runtime_py import convert

class Backend(QObject):

    field = Field()

    @Slot()
    def set_field(self, msg: Field) -> None:
        print(f"Qwidget {msg.goal_depth}")
        self.field = msg

    @Slot(result=list)
    def get_field(self):
        return (self.field.goal_depth, self.field.goal_width, self.field.width)

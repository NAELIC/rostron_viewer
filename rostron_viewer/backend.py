from PySide6.QtCore import QObject
from PySide6.QtCore import Slot
from rostron_interfaces.msg import Field, Ball

class Backend(QObject):

    field = Field()

    @Slot()
    def set_field(self, msg: Field) -> None:
        print(f"Qwidget {msg.goal_depth}")
        self.field = msg

    @Slot(result=list)
    def get_field(self):
        print(self.field.penalty_width)
        return (self.field.width, self.field.length, self.field.goal_depth, self.field.goal_width, self.field.penalty_width,  self.field.penalty_depth, self.field.boundary_width)
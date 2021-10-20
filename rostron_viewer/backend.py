from PySide6.QtCore import QObject
from PySide6.QtCore import Slot
from rostron_interfaces.msg import Field, Ball

class Backend(QObject):

    field = Field()
    ball = Ball()
    yellow = True

    @Slot()
    def set_field(self, msg: Field) -> None:
        self.field = msg

    @Slot(result=list)
    def get_field(self):
        return (self.field.width, self.field.length, self.field.goal_depth, self.field.goal_width, self.field.penalty_width,  self.field.penalty_depth, self.field.boundary_width)

    @Slot()
    def set_ball(self, msg: Ball) -> None:
        self.ball = msg
    
    @Slot(result=list)
    def get_ball(self):
        return (self.ball.position.x, self.ball.position.y)

    @Slot()
    def set_yellow(self, yellow):
        self.yellow = yellow
    
    @Slot(result=bool)
    def is_yellow(self):
        return self.yellow
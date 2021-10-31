from PySide6.QtCore import QObject
from PySide6.QtCore import Slot
from rostron_interfaces.msg import Field, Ball, Robots
from rostron_interfaces.srv import AddAnnotation, DeleteAnnotation


class Backend(QObject):

    field = Field()
    ball = Ball()
    allies = Robots()
    opponents = Robots()
    yellow = True
    annotations = {}

    @Slot()
    def set_field(self, msg: Field) -> None:
        self.field = msg

    @Slot(result='QVariant')
    def get_field(self):
        return {'width': self.field.width, 'length': self.field.length,  'goal_depth': self.field.goal_depth,   'goal_width': self.field.goal_width,  'penalty_width': self.field.penalty_width,  'penalty_depth': self.field.penalty_depth,  'boundary_width': self.field.boundary_width}

    @Slot()
    def set_ball(self, msg: Ball) -> None:
        self.ball = msg

    @Slot(result='QVariant')
    def get_ball(self):
        return {'x': self.ball.position.x, 'y': self.ball.position.y}

    @Slot()
    def set_yellow(self, yellow):
        self.yellow = yellow

    @Slot(result=bool)
    def is_yellow(self):
        return self.yellow

    @Slot()
    def set_allies(self, allies):
        self.allies = allies

    @Slot()
    def set_opponents(self, opponents):
        self.opponents = opponents

    @Slot(result='QVariant')
    def get_robots(self):
        msg = {'allies': [], 'opponents': []}

        for robot in self.allies.robots:
            if robot.active:
                msg['allies'].append(
                    {'id': robot.id, 'x': robot.pose.position.x, 'y': robot.pose.position.y, 'orientation': robot.pose.orientation.z})

        for robot in self.opponents.robots:
            if robot.active:
                msg['opponents'].append(
                    {'id': robot.id, 'x': robot.pose.position.x, 'y': robot.pose.position.y, 'orientation': robot.pose.orientation.z})
        return msg

    @Slot()
    def add_annotation(self, req: AddAnnotation.Request):
        self.annotations[req.id] = {
            'type': req.type, 'params': req.params}
    
    @Slot()
    def del_annotation(self, req : DeleteAnnotation.Request):
        self.annotations.pop(req.id)

    @Slot(result='QVariant')
    def get_annotations(self):
        return self.annotations

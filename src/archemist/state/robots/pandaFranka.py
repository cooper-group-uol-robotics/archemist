from src.archemist.state.robot import armRobot, RobotOpDescriptor, RobotOutputDescriptor


class PandaFranka(armRobot):
    def __init__(self, id: int):
        super().__init__(id)

''' ==== Robot Operation Descriptors ==== '''

class PandaMoveOpDescriptor(RobotOpDescriptor):
    def __init__(self, start_pos: str, end_pos: str):
        super().__init__(robotName=PandaFranka.__class__)
        self._start_pos = start_pos
        self._end_pos = end_pos

    @property
    def start_pos(self):
        return self._start_pos

    @property
    def send_pos(self):
        return self._end_pos

''' ==== Robot Output Descriptors ==== '''

class PandaOutputtDescriptor(RobotOutputDescriptor):
    def __init__(self, opName: str, success:bool):
        super().__init__(opName=opName, succes=success)
from archemist.state.robot import armRobot, RobotOpDescriptor, RobotOutputDescriptor,VialMoveOpDescriptor


class PandaFranka(armRobot):
    def __init__(self, id: int):
        super().__init__(id)

''' ==== Robot Operation Descriptors ==== '''

class PandaMoveOpDescriptor(VialMoveOpDescriptor):
    def __init__(self, start_pos: str, end_pos: str):
        super().__init__(robotName=PandaFranka.__name__, start_pos=start_pos, 
                         end_pos=end_pos, output=PandaOutputtDescriptor(self.__class__.__name__))

''' ==== Robot Output Descriptors ==== '''

class PandaOutputtDescriptor(RobotOutputDescriptor):
    def __init__(self, opName: str):
        super().__init__(opName=opName)

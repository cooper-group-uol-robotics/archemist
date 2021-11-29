from archemist.state.robot import armRobot, RobotOpDescriptor, RobotOutputDescriptor,VialMoveOpDescriptor
from archemist.util.location import Location


class PandaFranka(armRobot):
    def __init__(self, id: int):
        super().__init__(id)

''' ==== Robot Operation Descriptors ==== '''

# class PandaMoveOpDescriptor(VialMoveOpDescriptor):
#     def __init__(self, start_pos: Location, end_pos: Location):
#         self.robot_name = PandaFranka.__name__
#         super().__init__(start_pos=start_pos, 
#                          end_pos=end_pos, output=PandaOutputtDescriptor(self.__class__.__name__))

''' ==== Robot Output Descriptors ==== '''

# class PandaOutputtDescriptor(RobotOutputDescriptor):
#     def __init__(self):
#         super().__init__()

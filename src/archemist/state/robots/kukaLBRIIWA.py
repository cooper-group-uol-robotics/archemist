from archemist.state.robot import mobileRobot, SpecialJobOpDescriptor, RobotOutputDescriptor
from archemist.util import Location


class KukaLBRIIWA(mobileRobot):
    def __init__(self, id: int, saved_frames: list):
        super().__init__(id, saved_frames)


''' ==== Robot Operation Descriptors ==== '''

# class KukaMoveBaseOpDescriptor(TransportBatchOpDescriptor):
#     def __init__(self, robot_id: int, target_loc: Location, fine_localization: bool):
#         self.robot_name = KukaLBRIIWA.__name__
#         super().__init__(target_loc=target_loc, 
#                          output=KukaOutputtDescriptor(self.__class__.__name__))
#         self._fine_localization = fine_localization
#         self._robot_id = robot_id

#     @property
#     def fine_localization(self):
#         return self._fine_localization

#     @property
#     def robot_id(self):
#         return self._robot_id

# class KukaVialMoveOpDescriptor(VialMoveOpDescriptor):
#     def __init__(self, start_pos: Location, end_pos: Location):
#         self.robot_name = KukaLBRIIWA.__name__
#         super().__init__(start_pos=start_pos, 
#                          end_pos=end_pos, output=KukaOutputtDescriptor(self.__class__.__name__))

# class KukaRackMoveOpDescriptor(RackMoveOpDescriptor):
#     def __init__(self, start_pos: Location, end_pos: Location):
#         self.robot_name = KukaLBRIIWA.__name__
#         super().__init__(start_pos=start_pos,
#                          end_pos=end_pos, output=KukaOutputtDescriptor(self.__class__.__name__))

# class KukaCalibrateArmOpDescriptor(SpecialJobOpDescriptor):
#     def __init__(self, location: str):
#         self.robot_name = KukaLBRIIWA.__name__
#         super().__init__(job_name='ArmCalibrate', output=KukaOutputtDescriptor(self.__class__.__name__))
#         self._location = location

#     @property
#     def location(self):
#         return self._location



''' ==== Robot Output Descriptors ==== '''

# class KukaOutputtDescriptor(RobotOutputDescriptor):
#     def __init__(self):
#         super().__init__()

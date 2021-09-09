from archemist.state.robot import mobileRobot, RobotOpDescriptor, RobotOutputDescriptor
from archemist.state.station import Location


class KukaLBRIIWA(mobileRobot):
    def __init__(self, id: int):
        super().__init__(id)
        self._pose = None

    @property
    def pose(self):
        return self._pose

    def moveToPose(self, pose: str):
        self._pose = pose


''' ==== Robot Operation Descriptors ==== '''

class KukaMoveBaseOpDescriptor(RobotOpDescriptor):
    def __init__(self, robot_id: int, target_loc: Location, fine_localization: bool):
        super().__init__(robotName=KukaLBRIIWA.__class__)
        self._target_loc = target_loc
        self._fine_localization = fine_localization
        self._robot_id = robot_id

    @property
    def target_loc(self):
        return self._target_loc

    @property
    def fine_localization(self):
        return self._fine_localization

    @property
    def robot_id(self):
        return self._robot_id


class KukaMoveArmOpDescriptor(RobotOpDescriptor):
    def __init__(self, start_pos: str, end_pos: str):
        super().__init__(robotName=KukaLBRIIWA.__class__)
        self._start_pos = start_pos
        self._end_pos = end_pos

    @property
    def start_pos(self):
        return self._start_pos

    @property
    def end_pos(self):
        return self._end_pos

class KukaCalibrateArmOpDescriptor(RobotOpDescriptor):
    def __init__(self, location: str):
        super().__init__(robotName=KukaLBRIIWA.__class__)
        self._location = location

    @property
    def location(self):
        return self._location



''' ==== Robot Output Descriptors ==== '''

class PandaOutputtDescriptor(RobotOutputDescriptor):
    def __init__(self, opName: str, success:bool):
        super().__init__(opName=opName, succes=success)

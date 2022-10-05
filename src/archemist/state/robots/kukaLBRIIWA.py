from archemist.state.robot import MobileRobot, RobotTaskOpDescriptor, RobotOutputDescriptor, RobotOpDescriptor,RobotTaskType
from archemist.util import Location
from bson.objectid import ObjectId


class KukaLBRTask(RobotTaskOpDescriptor):
    def __init__(self, job_name: str, job_params: list, job_type: RobotTaskType, job_location: Location,  output: RobotOutputDescriptor = RobotOutputDescriptor()):
        super().__init__(job_name, job_params, job_type, job_location, output)

class KukaLBRMaintenanceTask(RobotTaskOpDescriptor):
    def __init__(self, job_name: str, job_params: list,  output: RobotOutputDescriptor = RobotOutputDescriptor()):
        super().__init__(job_name, job_params, RobotTaskType.OTHER, Location(-1,-1,''), output)

    def __str__(self) -> str:
        return f'{self.__class__.__name__} with task: {self._job_name}'

class KukaNAVTask(RobotOpDescriptor):
    def __init__(self, target_location: Location, fine_localisation: bool,  output: RobotOutputDescriptor = RobotOutputDescriptor()):
        super().__init__(output)
        self._target_location = target_location
        self._fine_localisation =  fine_localisation
    
    @property
    def target_location(self):
        return self._target_location

    @property
    def fine_localisation(self):
        return self._fine_localisation


class KukaLBRIIWA(MobileRobot):
    def __init__(self, db_name: str, robot_document: dict):
        super().__init__(db_name, robot_document)

    @classmethod
    def from_dict(cls, db_name: str, robot_document: dict):
        robot_document['location'] = {'node_id':-1, 'graph_id':-1, 'frame_name':''}
        return cls(db_name, robot_document)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        robot_dict = {'object_id':object_id}
        return cls(db_name, robot_dict)

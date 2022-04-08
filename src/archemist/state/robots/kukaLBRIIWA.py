from archemist.state.robot import mobileRobot, SpecialJobOpDescriptor, RobotOutputDescriptor
from archemist.util import Location
from bson.objectid import ObjectId


class KukaLBRTask(SpecialJobOpDescriptor):
    def __init__(self, job_name: str, job_params: list, job_location: Location, output: RobotOutputDescriptor):
        super().__init__(job_name, job_params, job_location, output=output)



class KukaLBRIIWA(mobileRobot):
    def __init__(self, db_name: str, robot_document: dict):
        super().__init__(db_name, robot_document)

    @classmethod
    def from_dict(cls, db_name: str, robot_document: dict):
        return cls(db_name, robot_document)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        robot_dict = {'object_id':object_id}
        return cls(db_name, robot_dict)

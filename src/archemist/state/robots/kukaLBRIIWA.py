from archemist.state.robot import MobileRobot, MobileRobotModel, RobotTaskOpDescriptor, RobotTaskOpDescriptorModel, RobotOpDescriptorModel, RobotOpDescriptor,RobotTaskType
from archemist.util import Location
from bson.objectid import ObjectId
from mongoengine import fields
from typing import List


class KukaLBRTask(RobotTaskOpDescriptor):
    def __init__(self, op_model: RobotTaskOpDescriptorModel):
        super().__init__(op_model)
    
    @classmethod
    def from_args(cls, name: str, type: RobotTaskType=RobotTaskType.MANIPULATION, parametrs: List[str]=[], 
                    location: Location=Location(), origin_station: ObjectId=None, related_batch_id: int=None):
        model = super().from_args(name,type,parametrs,location,origin_station,related_batch_id).model
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)
    
    

class KukaLBRMaintenanceTask(RobotTaskOpDescriptor):
    def __init__(self, op_model: RobotTaskOpDescriptorModel):
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str, parametrs: List[str]):
        model = super().from_args(name,RobotTaskType.OTHER,parametrs).model
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    def __str__(self) -> str:
        return f'{self.__class__.__name__} with task: {self._model.name}'

class KukaNAVTaskModel(RobotOpDescriptorModel):
    target_location = fields.DictField(required=True)
    fine_localisation = fields.BooleanField(default=False)

class KukaNAVTask(RobotOpDescriptor):
    def __init__(self, op_model: KukaNAVTaskModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, target_location: Location, fine_localisation: bool):
        model = KukaNAVTaskModel()
        model._type = cls.__name__
        model._module = cls.__module__
        model.target_location = target_location.to_dict()
        model.fine_localisation = fine_localisation
        return cls(model)
    
    @property
    def target_location(self):
        loc_dict = self._model.target_location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')

    @property
    def fine_localisation(self):
        return self._model.fine_localisation


class KukaLBRIIWA(MobileRobot):
    def __init__(self, robot_model: MobileRobotModel):
        self._model = robot_model

    @classmethod
    def from_dict(cls, robot_document: dict):
        model = MobileRobotModel()
        model._type = cls.__name__
        model._module = cls.__module__
        robot_document['location'] = {'node_id':-1, 'graph_id':-1, 'frame_name':''}
        cls._set_model_common_fields(robot_document, model)
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = MobileRobotModel.objects.get(id=object_id)
        return cls(model)

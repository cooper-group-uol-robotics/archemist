from archemist.core.models.robot_op_model import RobotTaskOpDescriptorModel
from archemist.core.state.robot import MobileRobot, RobotTaskOpDescriptor, RobotOpDescriptor,RobotTaskType
from archemist.core.util import Location
from .model import KukaLBRIIWAModel, KukaNAVTaskModel,KukaLBRTaskModel
from bson.objectid import ObjectId
from typing import Any, List
from archemist.core.persistence.object_factory import RobotFactory

class KukaLBRTask(RobotTaskOpDescriptor):
    def __init__(self, op_model: KukaLBRTaskModel):
       self._model = op_model
    
    @classmethod
    def from_args(cls, name: str, type: RobotTaskType=RobotTaskType.MANIPULATION, params: List[str]=[], 
                    location: Location=Location(), origin_station: ObjectId=None, related_batch_id: int=None, lock_robot: str=""):
        model = KukaLBRTaskModel()
        cls._set_model_common_fields(model)
        model._type = cls.__name__
        model._module = cls.__module__
        model.name = name
        model.task_type = type
        model.params = [str(param) for param in params]
        model.location = location.to_dict() if location is not None else None
        model.origin_station = origin_station
        model.related_batch_id = related_batch_id
        model.lock_robot = lock_robot
        return cls(model)
    
    @property
    def lock_robot(self) -> str:
        return self._model.lock_robot

class KukaLBRMaintenanceTask(RobotTaskOpDescriptor):
    def __init__(self, op_model: RobotTaskOpDescriptorModel):
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str, params: List[str]):
        model = super().from_args(name,RobotTaskType.OTHER,params).model
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    def __str__(self) -> str:
        return f'{self.__class__.__name__} with task: {self._model.name}'

class KukaNAVTask(RobotOpDescriptor):
    def __init__(self, op_model: KukaNAVTaskModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, target_location: Location, fine_localisation: bool):
        model = KukaNAVTaskModel()
        cls._set_model_common_fields(model)
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
    def __init__(self, robot_model: KukaLBRIIWAModel):
        self._model = robot_model

    @classmethod
    def from_dict(cls, robot_document: dict):
        model = KukaLBRIIWAModel()
        model._type = cls.__name__
        model._module = cls.__module__
        robot_document['location'] = {'node_id':-1, 'graph_id':-1, 'frame_name':''}
        cls._set_model_common_fields(robot_document, model)
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = KukaLBRIIWAModel.objects.get(id=object_id)
        return cls(model)
    
    @property
    def locked_to_station(self) -> bool:
        self._model.reload("locked_to_station")
        return self._model.locked_to_station
    
    @locked_to_station.setter
    def locked_to_station(self, locked: bool):
        self._model.update(locked_to_station=locked)

    def get_complete_op(self) -> Any:
        if self.is_assigned_op_complete():
            complete_op = RobotFactory.create_op_from_model(self._model.complete_op)
            if isinstance(complete_op, KukaLBRTask) and complete_op.lock_robot == "unlock":
                self.locked_to_station = False
        return super().get_complete_op()

from __future__ import annotations
from archemist.core.models.robot_op_model import RobotOpDescriptorModel,RobotTaskOpDescriptorModel
from archemist.core.util.enums import RobotTaskType
from archemist.core.util import Location
from bson.objectid import ObjectId
from datetime import datetime
from typing import List
import uuid


class RobotOpDescriptor:
    def __init__(self, op_model: RobotOpDescriptorModel) -> None:
        self._model = op_model
        
    @classmethod
    def _set_model_common_fields(cls, op_model: RobotOpDescriptorModel):
        op_model.uuid = uuid.uuid4()

    @property
    def model(self) -> RobotOpDescriptorModel:
        return self._model
    
    @property
    def uuid(self) -> uuid.UUID:
        return self._model.uuid

    @property
    def origin_station(self) -> ObjectId:
        return self._model.origin_station

    @origin_station.setter
    def origin_station(self, station_object_id: ObjectId) -> None:
        self._model.origin_station = station_object_id

    @property
    def related_batch_id(self) -> int:
        return self._model.related_batch_id

    @related_batch_id.setter
    def related_batch_id(self, batch_id: int) -> None:
        self._model.related_batch_id = batch_id

    @property
    def has_result(self) -> bool:
        return self._model.has_result

    @property
    def was_successful(self) -> bool:
        return self._model.was_successful

    @property
    def start_timestamp(self) -> datetime:
        return self._model.start_timestamp

    @property
    def end_timestamp(self) -> datetime:
        return self._model.end_timestamp

    @property
    def robot_stamp(self):
        return self._model.robot_stamp

    def add_start_timestamp(self):
        self._model.start_timestamp = datetime.now()

    def complete_op(self, robot_stamp: str, success: bool):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.robot_stamp = robot_stamp
        self._model.end_timestamp = datetime.now()

class RobotTaskOpDescriptor(RobotOpDescriptor):
    def __init__(self, op_model: RobotTaskOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, name: str, type: RobotTaskType=RobotTaskType.MANIPULATION, params: List[str]=[], 
                    location: Location=Location(), origin_station: ObjectId=None, related_batch_id: int=None):
        model = RobotTaskOpDescriptorModel()
        cls._set_model_common_fields(model)
        model._type = cls.__name__
        model._module = cls.__module__
        model.name = name
        model.task_type = type
        model.params = [str(param) for param in params]
        model.location = location.to_dict() if location is not None else None
        model.origin_station = origin_station
        model.related_batch_id = related_batch_id
        return cls(model)
    
    @property
    def name(self):
        return self._model.name

    @property
    def task_type(self):
        return self._model.task_type

    @property
    def params(self):
        return self._model.params

    @property
    def location(self):
        loc_dict = self._model.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')

    def __str__(self) -> str:
        return f'{self.__class__.__name__} with task: {self.name}, params: {self.params} @{self.location.get_map_coordinates()}'
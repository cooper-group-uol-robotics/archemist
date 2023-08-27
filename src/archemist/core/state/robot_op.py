from __future__ import annotations
from archemist.core.models.robot_op_model import RobotOpDescriptorModel,RobotTaskOpDescriptorModel
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.batch import Batch
from archemist.core.util.enums import RobotTaskType, OpResult
from archemist.core.util import Location
from bson.objectid import ObjectId
from datetime import datetime
from typing import List, Union
import uuid


class RobotOpDescriptor:
    def __init__(self, op_model: Union[RobotOpDescriptorModel, ModelProxy]) -> None:
        if isinstance(op_model, ModelProxy):
            self._model_proxy = op_model
        else:
            self._model_proxy = ModelProxy(op_model)
        
    @classmethod
    def _set_model_common_fields(cls, op_model: RobotOpDescriptorModel):
        op_model.uuid = uuid.uuid4()

    @classmethod
    def construct_op(cls, **kwargs):
        model = RobotOpDescriptorModel()
        cls._set_model_common_fields(model, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def model(self) -> RobotOpDescriptorModel:
        return self._model_proxy.model
    
    @property
    def uuid(self) -> uuid.UUID:
        return self._model_proxy.uuid

    @property
    def requested_by(self) -> ObjectId:
        return self._model_proxy.requested_by

    @requested_by.setter
    def requested_by(self, station_id: ObjectId) -> None:
        self._model_proxy.requested_by = station_id

    @property
    def executed_by(self) -> ObjectId:
        return self._model_proxy.executed_by

    @property
    def related_batch(self) -> Batch:
        if self._model_proxy.related_batch:
            return Batch(self._model_proxy.related_batch)

    @related_batch.setter
    def related_batch(self, batch: Batch) -> None:
        self._model_proxy.related_batch = batch.model

    @property
    def has_result(self) -> bool:
        return self._model_proxy.has_result

    @property
    def result(self) -> OpResult:
        return self._model_proxy.result

    @property
    def start_timestamp(self) -> datetime:
        return self._model_proxy.start_timestamp
    
    @start_timestamp.setter
    def start_timestamp(self, new_start_timestamp: datetime):
        self._model_proxy.start_timestamp = new_start_timestamp

    @property
    def end_timestamp(self) -> datetime:
        return self._model_proxy.end_timestamp
    
    @end_timestamp.setter
    def end_timestamp(self, new_end_timestamp: datetime):
        self._model_proxy.end_timestamp = new_end_timestamp

    def add_start_timestamp(self):
        self._model_proxy.start_timestamp = datetime.now()

    def complete_op(self, robot_id: ObjectId, result: OpResult):
        self._model_proxy.has_result = True
        self._model_proxy.result = result
        self._model_proxy.executed_by = robot_id
        self._model_proxy.end_timestamp = datetime.now()

class RobotTaskOpDescriptor(RobotOpDescriptor):
    def __init__(self, op_model: Union[RobotTaskOpDescriptorModel, ModelProxy]) -> None:
        if isinstance(op_model, ModelProxy):
            self._model_proxy = op_model
        else:
            self._model_proxy = ModelProxy(op_model)

    @classmethod
    def from_args(cls, name: str, type: RobotTaskType=RobotTaskType.MANIPULATION, params: List[str]=[], 
                    location: Location=Location(), origin_station_id: ObjectId=None, related_batch: Batch=None):
        model = RobotTaskOpDescriptorModel()
        cls._set_model_common_fields(model)
        model._type = cls.__name__
        model._module = cls.__module__
        model.name = name
        model.task_type = type
        model.params = [str(param) for param in params]
        model.location = location.to_dict() if location is not None else None
        model.requested_by = origin_station_id
        model.related_batch = related_batch.model if related_batch else None
        model.save()
        return cls(model)
    
    @property
    def name(self) -> str:
        return self._model_proxy.name

    @property
    def task_type(self) -> RobotTaskType:
        return self._model_proxy.task_type

    @property
    def params(self) -> List[str]:
        return self._model_proxy.params

    @property
    def location(self) -> Location:
        loc_dict = self._model_proxy.location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])

    def __str__(self) -> str:
        return f'{self.__class__.__name__} with task: {self.name}, params: {self.params} @{self.location.get_map_coordinates()}'
from __future__ import annotations
from archemist.core.models.robot_op_model import RobotOpDescriptorModel,RobotTaskOpDescriptorModel, RobotNavOpDescriptorModel, RobotWaitOpDescriptorModel, RobotMaintenanceOpDescriptorModel
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import RobotTaskType, OpResult
from archemist.core.util import Location
from bson.objectid import ObjectId
from datetime import datetime
from typing import List, Union, Dict
import uuid


class RobotOpDescriptor:
    def __init__(self, op_model: Union[RobotOpDescriptorModel, ModelProxy]) -> None:
        if isinstance(op_model, ModelProxy):
            self._model_proxy = op_model
        else:
            self._model_proxy = ModelProxy(op_model)
        
    @classmethod
    def _set_model_common_fields(cls, op_model: RobotOpDescriptorModel, target_robot: str):
        op_model.uuid = uuid.uuid4()
        op_model.target_robot = target_robot
        op_model._type = cls.__name__
        op_model._module = cls.__module__

    @classmethod
    def from_args(cls):
        model = RobotOpDescriptorModel()
        cls._set_model_common_fields(model, "Robot")
        model.save()
        return cls(model)

    @property
    def model(self) -> RobotOpDescriptorModel:
        return self._model_proxy.model
    
    @property
    def uuid(self) -> uuid.UUID:
        return self._model_proxy.uuid

    @property
    def target_robot(self) -> str:
        return self._model_proxy.target_robot

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
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str, target_robot: str, type: RobotTaskType=RobotTaskType.MANIPULATION, params: Dict={}, **kwargs):
        model = RobotTaskOpDescriptorModel()
        cls._set_model_common_fields(model, target_robot)
        model.name = name
        model.task_type = type
        model.params = params
        location = kwargs.get("location")
        model.location = location.to_dict() if location is not None else None
        model.requested_by = kwargs.get("origin_station_id")
        related_batch = kwargs.get("related_batch")
        related_lot = kwargs.get("related_lot")
        model.related_batch = related_batch.model if related_batch else None
        model.related_lot = related_lot.model if related_lot else None
        model.save()
        return cls(model)
    
    @property
    def name(self) -> str:
        return self._model_proxy.name

    @property
    def task_type(self) -> RobotTaskType:
        return self._model_proxy.task_type

    @property
    def params(self) -> Dict:
        return self._model_proxy.params

    @property
    def location(self) -> Location:
        loc_dict = self._model_proxy.location
        if loc_dict:
            return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name=loc_dict['frame_name'])
        
    @property
    def related_batch(self) -> Batch:
        if self._model_proxy.related_batch:
            return Batch(self._model_proxy.related_batch)

    @related_batch.setter
    def related_batch(self, batch: Batch) -> None:
        self._model_proxy.related_batch = batch.model

    @property
    def related_lot(self) -> Lot:
        if self._model_proxy.related_lot:
            return Lot(self._model_proxy.related_lot)

    @related_lot.setter
    def related_lot(self, lot: Lot) -> None:
        self._model_proxy.related_lot = lot.model

    def __str__(self) -> str:
        params = self.params if self.params else None
        location = f"{self.location.get_map_coordinates()}" if self.location else ""
        return f'{self.__class__.__name__} -> name: {self.name} - target:{self.target_robot}\
              - params: {params} - location: {location}'
    
class RobotMaintenanceOpDescriptor(RobotOpDescriptor):
    def __init__(self, op_model: Union[RobotMaintenanceOpDescriptorModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str, target_robot: str, target_robot_id: int, params: Dict={}):
        model = RobotMaintenanceOpDescriptorModel()
        cls._set_model_common_fields(model, target_robot)
        model.name = name
        model.target_robot_id = target_robot_id
        model.params = params
        model.requested_by = ObjectId(b"maintainer  ")
        model.save()
        return cls(model)
    
    @property
    def name(self) -> str:
        return self._model_proxy.name
    
    @property
    def target_robot_id(self) -> int:
        return self._model_proxy.target_robot_id

    @property
    def params(self) -> Dict:
        return self._model_proxy.params

    def __str__(self) -> str:
        params = self.params if self.params else None
        return f'{self.__class__.__name__} -> name: {self.name}\
              - target:{self.target_robot}_{self.target_robot_id} - params: {params}'
    
class RobotNavOpDescriptor(RobotOpDescriptor):
    def __init__(self, op_model: Union[RobotNavOpDescriptorModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str, target_robot: str, target_location: Location, params: Dict={}, **kwargs):
        model = RobotNavOpDescriptorModel()
        cls._set_model_common_fields(model, target_robot)
        model.name = name
        model.target_location = target_location.to_dict()
        model.params = params
        model.requested_by = kwargs.get("origin_station_id")
        model.save()
        return cls(model)
    
    @property
    def name(self) -> str:
        return self._model_proxy.name
    
    @property
    def params(self) -> Dict:
        return self._model_proxy.params
    
    @property
    def target_location(self) -> Location:
        loc_dict = self._model_proxy.target_location
        return Location(node_id=loc_dict['node_id'],graph_id=loc_dict['graph_id'], frame_name='')
    
    def __str__(self) -> str:
        params = self.params if self.params else None
        return f'{self.__class__.__name__} -> name: {self.name} - goal_location: {self.target_location}\
              - params: {params} - target:{self.target_robot}'
    
class RobotWaitOpDescriptor(RobotOpDescriptor):
    def __init__(self, op_model: Union[RobotWaitOpDescriptorModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_robot: str, timeout: int, **kwargs):
        model = RobotWaitOpDescriptorModel()
        cls._set_model_common_fields(model, target_robot)
        model.timeout = timeout
        model.requested_by = kwargs.get("origin_station_id")
        model.save()
        return cls(model)
    
    @property
    def timeout(self) -> int:
        return self._model_proxy.timeout
    
    def __str__(self) -> str:
        return f'{self.__class__.__name__} -> requested_by: {self.requested_by} - target:{self.target_robot}\
              - timeout: {self.timeout}'

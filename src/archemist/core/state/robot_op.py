from archemist.core.models.robot_op_model import (RobotOpModel,
                                                  RobotTaskOpModel,
                                                  CollectBatchOpModel,
                                                  DropBatchOpModel,
                                                  RobotNavOpModel,
                                                  RobotWaitOpModel,
                                                  RobotMaintenanceOpModel)
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import OpOutcome
from archemist.core.util import Location
from bson.objectid import ObjectId
from datetime import datetime
from typing import Union, Dict


class RobotOp:
    def __init__(self, op_model: Union[RobotOpModel, ModelProxy]):
        if isinstance(op_model, ModelProxy):
            self._model_proxy = op_model
        else:
            self._model_proxy = ModelProxy(op_model)
        
    @classmethod
    def _set_model_common_fields(cls, op_model: RobotOpModel, target_robot: str):
        op_model.target_robot = target_robot
        op_model._type = cls.__name__
        op_model._module = cls.__module__

    @classmethod
    def from_args(cls):
        model = RobotOpModel()
        cls._set_model_common_fields(model, "Robot")
        model.save()
        return cls(model)

    @property
    def model(self) -> RobotOpModel:
        return self._model_proxy.model
    
    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id

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
    def outcome(self) -> OpOutcome:
        return self._model_proxy.outcome

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

    def complete_op(self, robot_id: ObjectId, outcome: OpOutcome):
        self._model_proxy.outcome = outcome
        self._model_proxy.executed_by = robot_id
        self._model_proxy.end_timestamp = datetime.now()

    def __eq__(self, __value: object) -> bool:
        return self.object_id == __value.object_id

class RobotTaskOp(RobotOp):
    def __init__(self, op_model: Union[RobotTaskOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str,
                  target_robot: str,
                  params: Dict={},
                  target_location: Location=None,
                  requested_by: ObjectId()=None,
                  target_batch: Batch=None
                  ):
        model = RobotTaskOpModel()
        cls._set_model_common_fields(model, target_robot)
        model.name = name
        model.params = params
        if target_location:
            model.target_location = target_location.model
        model.requested_by = requested_by
        model.target_batch = target_batch.model if target_batch else None
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
            return Location(self._model_proxy.target_location)
        
    @target_location.setter
    def target_location(self, new_location: Location):
        if isinstance(new_location, Location):
            self._model_proxy.target_location = new_location.model
        else:
            raise ValueError
        
    @property
    def target_batch(self) -> Batch:
        if self._model_proxy.target_batch:
            return Batch(self._model_proxy.target_batch)

    @target_batch.setter
    def target_batch(self, batch: Batch) -> None:
        self._model_proxy.target_batch = batch.model

    @property
    def related_lot(self) -> Lot:
        target_batch = self.target_batch
        if target_batch:
            return Lot.from_object_id(target_batch.parent_lot_id)

    def __str__(self) -> str:
        params = dict(self.params) if self.params else None
        return f'{self.__class__.__name__} - name: {self.name} - target:{self.target_robot}\
              - params: {params} - location: {self.target_location}'
    
class CollectBatchOp(RobotTaskOp):
    def __init__(self, op_model: Union[CollectBatchOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str,
                  target_robot: str,
                  params: Dict={},
                  target_location: Location=None,
                  requested_by: ObjectId()=None,
                  target_batch: Batch=None
                  ):
        model = CollectBatchOpModel()
        cls._set_model_common_fields(model, target_robot)
        model.name = name
        model.params = params
        if target_location:
            model.target_location = target_location.model
        model.requested_by = requested_by
        model.target_batch = target_batch.model if target_batch else None
        model.save()
        return cls(model)

    @property
    def target_onboard_slot(self) -> int:
        return self._model_proxy.target_onboard_slot

    @target_onboard_slot.setter
    def target_onboard_slot(self, onboard_slot: int):
        self._model_proxy.target_onboard_slot = onboard_slot

class DropBatchOp(RobotTaskOp):
    def __init__(self, op_model: Union[DropBatchOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str,
                  target_robot: str,
                  params: Dict={},
                  target_location: Location=None,
                  requested_by: ObjectId()=None,
                  target_batch: Batch=None
                  ):
        model = DropBatchOpModel()
        cls._set_model_common_fields(model, target_robot)
        model.name = name
        model.params = params
        if target_location:
            model.target_location = target_location.model
        model.requested_by = requested_by
        model.target_batch = target_batch.model if target_batch else None
        model.save()
        return cls(model)

    @property
    def onboard_collection_slot(self) -> int:
        return self._model_proxy.onboard_collection_slot

    @onboard_collection_slot.setter
    def onboard_collection_slot(self, onboard_slot: int):
        self._model_proxy.onboard_collection_slot = onboard_slot
    
class RobotMaintenanceOp(RobotOp):
    def __init__(self, op_model: Union[RobotMaintenanceOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str, target_robot: str, target_robot_id: int, params: Dict={}):
        model = RobotMaintenanceOpModel()
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
    
class RobotNavOp(RobotOp):
    def __init__(self, op_model: Union[RobotNavOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, name: str, target_robot: str, target_location: Location, params: Dict={}, requested_by: ObjectId()=None):
        model = RobotNavOpModel()
        cls._set_model_common_fields(model, target_robot)
        model.name = name
        if target_location:
            model.target_location = target_location.model
        model.params = params
        model.requested_by = requested_by
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
            return Location(self._model_proxy.target_location)
        
    @target_location.setter
    def target_location(self, new_location: Location):
        if isinstance(new_location, Location):
            self._model_proxy.target_location = new_location.model
        else:
            raise ValueError
    
    def __str__(self) -> str:
        params = dict(self.params) if self.params else None
        return f'{self.__class__.__name__} - name: {self.name} - goal_location: {self.target_location}\
              - params: {params} - target:{self.target_robot}'
    
class RobotWaitOp(RobotOp):
    def __init__(self, op_model: Union[RobotWaitOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_robot: str, timeout: int, requested_by: ObjectId()=None):
        model = RobotWaitOpModel()
        cls._set_model_common_fields(model, target_robot)
        model.timeout = timeout
        model.requested_by = requested_by
        model.save()
        return cls(model)
    
    @property
    def timeout(self) -> int:
        return self._model_proxy.timeout
    
    def __str__(self) -> str:
        return f'{self.__class__.__name__} -> requested_by: {self.requested_by} - target:{self.target_robot}\
              - timeout: {self.timeout}'

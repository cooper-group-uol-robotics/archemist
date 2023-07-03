from __future__ import annotations
from typing import Union
from datetime import datetime
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.models.station_op_model import StationOpDescriptorModel
import uuid
from bson.objectid import ObjectId

class StationOpDescriptor:
    def __init__(self, station_op_model: Union[StationOpDescriptorModel, ModelProxy]) -> None:
        if isinstance(station_op_model, ModelProxy):
            self._model_proxy = station_op_model
        else:
            self._model_proxy = ModelProxy(station_op_model)

    @classmethod
    def _set_model_common_fields(cls, op_model: StationOpDescriptorModel, associated_station: str, **kwargs):
        op_model.uuid = uuid.uuid4()
        op_model.associated_station = associated_station

    @classmethod
    def construct_op(cls, **kwargs):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station="Station", **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def model(self):
        return self._model_proxy.model
    
    @property
    def associated_station(self) -> str:
        return self._model_proxy.associated_station

    @property
    def uuid(self) -> uuid.UUID:
        return self._model_proxy.uuid
    
    @property
    def requested_by(self) -> ObjectId:
        return self._model_proxy.requested_by

    @requested_by.setter
    def requested_by(self, station_id: ObjectId):
        self._model_proxy.requested_by = station_id

    @property
    def has_result(self) -> bool:
        return self._model_proxy.has_result

    @property
    def was_successful(self) -> bool:
        return self._model_proxy.was_successful

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

    def complete_op(self, success: bool, **kwargs):
        self._model_proxy.has_result = True
        self._model_proxy.was_successful = success
        self._model_proxy.end_timestamp = datetime.now()
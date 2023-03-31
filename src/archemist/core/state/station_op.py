from __future__ import annotations
from datetime import datetime
from archemist.core.models.station_op_model import StationOpDescriptorModel
import uuid
from bson.objectid import ObjectId

class StationOpDescriptor:
    def __init__(self, stationOpModel: StationOpDescriptorModel) -> None:
        self._model = stationOpModel

    @classmethod
    def _set_model_common_fields(cls, op_model: StationOpDescriptorModel, associated_station: str, **kwargs):
        op_model.uuid = uuid.uuid4()
        op_model.associated_station = associated_station

    @property
    def model(self):
        return self._model
    
    @property
    def associated_station(self) -> str:
        return self._model.associated_station

    @property
    def uuid(self) -> uuid.UUID:
        return self._model.uuid
    
    @property
    def requested_by(self) -> ObjectId:
        return self._model.requested_by

    @property
    def has_result(self) -> bool:
        return self._model.has_result

    @property
    def was_successful(self) -> bool:
        return self._model.was_successful

    @property
    def start_timestamp(self):
        return self._model.start_timestamp

    @property
    def end_timestamp(self):
        return self._model.end_timestamp
    
    def add_request_info(self, station_id: ObjectId):
        self._model.requested_by = station_id

    def add_start_timestamp(self):
        self._model.start_timestamp = datetime.now()

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()

    def copy_stamps(self, other_op: StationOpDescriptor):
        self._model.uuid = other_op.uuid
        self._model.requested_by = other_op.requested_by
        self._model.start_timestamp = other_op._model.start_timestamp
        if other_op.has_result:
            self._model.has_result = True
            self._model.was_successful = other_op.was_successful
            self._model.end_timestamp = other_op._model.end_timestamp
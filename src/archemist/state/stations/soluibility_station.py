from enum import Enum
from archemist.state.station import Station,StationModel,StationOpDescriptor, StationOpDescriptorModel
from typing import List
from archemist.state.material import Liquid, Solid
from bson.objectid import ObjectId
from mongoengine import fields
from datetime import datetime

class TurbidityState(Enum):
    DISSOLVED = 0
    PARTIALLY_DISSOLVED = 1
    UNDISSOLVED = 2


''' ==== Station Description ==== '''
class SolubilityStation(Station):
    def __init__(self, station_model: StationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_document: dict, liquids: List[Liquid], solids: List[Solid]):
        model = StationModel()
        cls._set_model_common_fields(station_document,model)
        model._type = cls.__name__
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = StationModel.objects.get(id=object_id)
        return cls(model)

''' ==== Station Operation Descriptors ==== '''
class SolubilityOpDescriptorModel(StationOpDescriptorModel):
    turbidity_state = fields.EnumField(TurbidityState)

class SolubilityOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: SolubilityOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls):
        model = SolubilityOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def turbidity_state(self) -> TurbidityState:
        if self._model.has_result and self._model.was_successful:
            return self._model.turbidity_state

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'turbidity_state' in kwargs:
            self._model.turbidity_state = kwargs['turbidity_state']
        else:
            pass #print('missing read weight!!')

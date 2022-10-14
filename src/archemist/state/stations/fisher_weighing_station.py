from bson.objectid import ObjectId
from archemist.state.station import Station,StationModel,StationOpDescriptorModel,StationOpDescriptor
from typing import List
from archemist.state.material import Liquid, Solid
from mongoengine import fields
from datetime import datetime

''' ==== Station Description ==== '''
class FisherWeightingStation(Station):
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
class FisherWeightOpDescriptorModel(StationOpDescriptorModel):
    weight = fields.FloatField(min_value=0)

class FisherWeightOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: FisherWeightOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls):
        model = FisherWeightOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def weight(self) -> float:
        if self._model.has_result and self._model.was_successful:
            return self._model.weight

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'weight' in kwargs:
            self._model.weight = kwargs['weight']
        else:
            pass #print('missing read weight!!')

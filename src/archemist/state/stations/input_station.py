from archemist.state.station import Station,StationModel,StationOpDescriptorModel,StationOpDescriptor
from archemist.state.material import Liquid,Solid
from typing import List
from bson.objectid import ObjectId


class InputStation(Station):
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

class InputStationPickupOp(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)
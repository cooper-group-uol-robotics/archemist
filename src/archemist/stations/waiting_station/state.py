from .model import WaitingStationModel, WaitingStationOpDescriptorModel
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from typing import List, Any, Dict
from archemist.core.state.material import Liquid, Solid
from datetime import datetime


''' ==== Station Description ==== '''
class WaitingStation(Station):
    def __init__(self, station_model: WaitingStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = StationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)
    
    def has_free_batch_capacity(self) -> bool: #TODO this need to change to accept more batches
        return super().has_free_batch_capacity()
        

''' ==== Station Operation Descriptors ==== '''
class WaitingOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: WaitingStationOpDescriptorModel)-> None:
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = WaitingStationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=WaitingStation.__name__, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        model.duration = int(kwargs['duration'])
        return cls(model)

    @property
    def duration(self) -> int:
        return self._model.duration
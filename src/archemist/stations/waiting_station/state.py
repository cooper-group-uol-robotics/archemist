from .model import WaitingStationModel, WaitingStationOpDescriptorModel
from archemist.core.models.station_model import StationModel
from archemist.core.state.station import Station
from archemist.core.util.enums import TimeUnit
from archemist.core.state.station_op import StationOpDescriptor
from typing import List, Dict
from archemist.core.state.material import Liquid, Solid


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
        time_unit = kwargs.get('time_unit')
        if time_unit is None or time_unit == "sec":
            model.time_unit = TimeUnit.SECONDS
        elif time_unit == "min":
            model.time_unit = TimeUnit.MINUTES
        elif time_unit == "hour":
            model.time_unit = TimeUnit.HOURS
        return cls(model)

    @property
    def duration(self) -> int:
        return self._model.duration
    
    @property
    def time_unit(self) -> TimeUnit:
        return self._model.time_unit
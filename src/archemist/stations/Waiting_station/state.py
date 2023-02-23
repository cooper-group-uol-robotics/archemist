from .model import WaitingStationStatus, WaitingStationModel, WaitingStationOpDescriptorModel
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from typing import List, Any, Dict
from archemist.core.state.material import Liquid, Solid
from datetime import datetime


''' ==== Station Description ==== '''
class WaitingStation(Station):
    def __init__(self, station_model: StationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = StationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)
    
    @property
    def status(self) -> WaitingStationStatus:
        self._model.reload('machine_status')
        return self._model.machine_status
    
    @status.setter
    def status(self, new_status: WaitingStationStatus):
        self._model.update(machine_status=new_status)

    def assign_station_op(self, stationOp: Any): #TBC
        pass

    def complete_assigned_station_op(self, success: bool, **kwargs): #TBC
        pass

''' ==== Station Operation Descriptors ==== '''
class WaitingOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: WaitingStationOpDescriptorModel)-> None:
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = WaitingStationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        model.duration = int(kwargs['duration'])
        return cls(model)

    @property
    def duration(self) -> int:
        return self._model.duration
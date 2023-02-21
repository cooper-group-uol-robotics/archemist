from .model import WaitingStationOpDescriptorModel
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
    def status(self) -> int:
        self._model.reload('Number_of_racks_occupied')
        return self._model.Number_of_racks_occupied


''' ==== Station Operation Descriptors ==== '''
class WaitingOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def station_waiting_complete

    @status.setter
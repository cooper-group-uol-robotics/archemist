from .model import WaitingStationStatus, WaitingStationModel, WaitingStationAvailabity, WaitingStationOpDescriptorModel
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
    
    @property
    def status(self) -> WaitingStationStatus:
        self._model.reload('station_status')
        return self._model.station_status
    
    @status.setter
    def status(self, new_status: WaitingStationStatus):
        self._model.update(station_status=new_status)

    @property
    def availability(self) -> WaitingStationAvailabity:
        self._model.reload('station_availability')
        return self._model.station_availability
    
    @availability.setter
    def availability(self, new_availability: WaitingStationAvailabity):
        self._model.update(station_availability=new_availability)

    def assign_station_op(self, stationOp: Any): #TBC
        if isinstance(stationOp, WaitingOpDescriptor):
            self.status = WaitingStationStatus.Batch_waiting
        super().assign_station_op(stationOp)

    def complete_assigned_station_op(self, success: bool, **kwargs): #TBC
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, WaitingOpDescriptor):
            self.status = WaitingStationStatus.Not_waiting
        super().complete_assigned_station_op(success, **kwargs)
    
    def update_batch_count(self, BatchCount:int): #to increment number of slots
        if BatchCount == 3 and self._model.current_occupancy <= (self._model.batch_capacity-BatchCount):
            self.availability = WaitingStationAvailabity.Available
            self._model.current_occupancy += BatchCount
        elif BatchCount == 3 and self._model.current_occupancy >(self._model.batch_capacity-BatchCount):
            self.availability = WaitingStationAvailabity.Not_available
        elif BatchCount == 0:
            self._model.current_occupancy -= 3
            self.availability = WaitingStationAvailabity.Available
        

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
    

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)
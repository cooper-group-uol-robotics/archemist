from .model import FiltrationStationModel, FiltrationStatus
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from typing import List, Any, Dict
from archemist.core.state.material import Liquid, Solid
from datetime import datetime


''' ==== Station Description ==== '''
class FiltrationStation(Station):
    def __init__(self, station_model: FiltrationStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = FiltrationStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)
    
    @property
    def status(self) -> FiltrationStatus:
        self._model.reload('machine_status')
        return self._model.machine_status
    
    @status.setter
    def status(self, new_status: FiltrationStatus):
        self._model.update(machine_status=new_status)
    
    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, BaseValveOpenOpDescriptor):
            self.status = FiltrationStatus.BASEVALVE_OPEN
        elif isinstance(current_op, VacuumOpenOpDescriptor):
            self.status = FiltrationStatus.VACUUMING_OPEN
        elif isinstance(current_op, DrainValveOpenOpDescriptor):
            self.status = FiltrationStatus.DRAINING_OPEN
        elif isinstance(current_op, IdleOpDescriptor):
            self.status = FiltrationStatus.STOP

        super().complete_assigned_station_op(success, **kwargs)

    
''' ==== Station Operation Descriptors ==== '''
class BaseValveOpenOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: StationOpDescriptorModel):
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class BaseValveCloseOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: StationOpDescriptorModel):
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)
    
class VacuumOpenOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: StationOpDescriptorModel):
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class VacuumCloseOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: StationOpDescriptorModel):
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class DrainValveOpenOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: StationOpDescriptorModel):
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class DrainValveCloseOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: StationOpDescriptorModel):
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class IdleOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: StationOpDescriptorModel):
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)


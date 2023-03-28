from .model import OptimaxStatus, OptimaxStationModel, OptimaxOpDescriptorModel
from archemist.core.models.station_model import StationModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid,Solid
from typing import Dict, List, Any
from datetime import datetime


''' ==== Station Description ==== '''
class OptimaxStation(Station):
    def __init__(self, station_model: OptimaxStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = OptimaxStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)
    
    @property
    def status(self) -> OptimaxStatus:
        self._model.reload('machine_status')
        return self._model.machine_status
    
    @status.setter
    def status(self, new_status: OptimaxStatus):
        self._model.update(machine_status=new_status)

    @property
    def current_temperature(self) -> int:
        self._model.reload('current_temperature')
        return self._model.current_temperature

    @current_temperature.setter
    def current_temperature(self, new_temp: int):
        self._model.update(current_temperature=new_temp)
    
    @property
    def current_stirring_speed(self) -> int:
        self._model.reload('current_stirring_speed')
        return self._model.current_stirring_speed

    @current_stirring_speed.setter
    def current_stirring_speed(self, new_speed: int):
        self._model.update(current_stirring_speed=new_speed)

    def assign_station_op(self, stationOp: Any):
        if isinstance(stationOp, OptimaxTempStirringOpDescriptor):
            self.status = OptimaxStatus.T_AND_S
        elif isinstance(stationOp, OptimaxTempOpDescriptor):
            self.status = OptimaxStatus.TEMP_CONTROL
        elif isinstance(stationOp, OptimaxStirringOpDescriptor):
            self.status = OptimaxStatus.STIRRING
        super().assign_station_op(stationOp)
    
    def complete_assigned_station_op(self, success: bool, **kwargs):
        self._model.update(unset__target_temperature=True)
        self._model.update(unset__target_stirring_speed=True)
        self._model.update(unset__target_duration=True)
        self._model.update(unset__mode=True)
        super().complete_assigned_station_op(success, **kwargs)

''' ==== Station Operation Descriptors ==== '''    
class OptimaxTempStirringOpDescriptor(OptimaxOpDescriptorModel):
    def __init__(self, op_model: OptimaxOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = OptimaxOpDescriptorModel()
        model.target_temperature = int(kwargs['temperature'])
        model.target_stirring_speed = int(kwargs['stirring_speed'])
        model.target_duration = float(kwargs['duration'])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def target_temperature(self) -> int:
        return self._model.target_temperature

    @property
    def target_stirring_speed(self) -> int:
        return self._model.target_stirring_speed

    @property
    def target_duration(self) -> int:
        return self._model.target_duration

class OptimaxTempOpDescriptor(OptimaxOpDescriptorModel):
    def __init__(self, op_model: OptimaxOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = OptimaxOpDescriptorModel()
        model.target_temperature = int(kwargs['temperature'])
        model.target_duration = float(kwargs['duration'])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def target_temperature(self) -> int:
        return self._model.target_temperature

    @property
    def target_duration(self) -> int:
        return self._model.target_duration

class OptimaxStirringOpDescriptor(OptimaxOpDescriptorModel):
    def __init__(self, op_model: OptimaxOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = OptimaxOpDescriptorModel()
        model.target_stirring_speed = int(kwargs['stirring_speed'])
        model.target_duration = float(kwargs['duration'])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def target_stirring_speed(self) -> int:
        return self._model.target_stirring_speed

    @property
    def target_duration(self) -> int:
        return self._model.target_duration

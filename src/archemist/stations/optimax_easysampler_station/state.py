from .model import SynthesisStatus, SynthesisStationModel, OptimaxOpDescriptorModel, OptimaxSamplingOpDescriptorModel, LcmsOpDescriptorModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.models.station_model import StationModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid,Solid
from typing import Dict, List, Any
from datetime import datetime


''' ==== Station Description ==== '''
class SynthesisStation(Station):
    def __init__(self, station_model: SynthesisStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = SynthesisStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)
    
    @property
    def status(self) -> SynthesisStatus:
        self._model.reload('machine_status')
        return self._model.machine_status
    
    @status.setter
    def status(self, new_status: SynthesisStatus):
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
            self.status = SynthesisStatus.TEMP_AND_STIRRING
        elif isinstance(stationOp, OptimaxTempOpDescriptor):
            self.status = SynthesisStatus.TEMP_CONTROL
        elif isinstance(stationOp, OptimaxStirringOpDescriptor):
            self.status = SynthesisStatus.STIRRING
        elif isinstance(stationOp, LcmsOpDescriptor):
            self.status = SynthesisStatus.ANALYSIS
        super().assign_station_op(stationOp)
    

''' ==== Station Operation Descriptors ==== '''    
class OptimaxTempStirringOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: OptimaxOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = OptimaxOpDescriptorModel()
        model.temperature = int(kwargs['temperature'])
        model.temp_duration = int(kwargs['temp_duration'])
        model.stir_speed = int(kwargs['stir_speed'])
        model.stir_duration = int(kwargs['stir_duration'])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def temperature(self) -> int:
        return self._model.temperature

    @property
    def temp_duration(self) -> int:
        return self._model.temp_duration
    
    @property
    def stir_speed(self) -> int:
        return self._model.stir_speed
    
    @property
    def stir_duration(self) -> int:
        return self._model.stir_duration
    

class OptimaxTempOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: OptimaxOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = OptimaxOpDescriptorModel()
        model.temperature = int(kwargs['temperature'])
        model.temp_duration = float(kwargs['temp_duration'])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def temperature(self) -> int:
        return self._model.temperature

    @property
    def temp_duration(self) -> int:
        return self._model.temp_duration
    
class OptimaxStirringOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: OptimaxOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = OptimaxOpDescriptorModel()
        model.stir_speed = int(kwargs['stir_speed'])
        model.stir_duration = float(kwargs['stir_duration'])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def stir_speed(self) -> int:
        return self._model.stir_speed
    
    @property
    def stir_duration(self) -> int:
        return self._model.stir_duration
    
class OptimaxSamplingOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: OptimaxSamplingOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = OptimaxSamplingOpDescriptorModel()
        model.dilution = int(kwargs['dilution'])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def dilution(self) -> int:
        return self._model.dilution
    
class LcmsOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: LcmsOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = LcmsOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def concentration(self) -> int:
        return self._model.concentration_result

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'concentration' in kwargs:
            self._model.concentration_result = kwargs['concentration']
        else:
            pass

class ParacetamolSynthesisOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)
    
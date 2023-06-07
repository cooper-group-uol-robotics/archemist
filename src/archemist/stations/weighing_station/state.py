from .model import WeightOpDescriptorModel, BalanceDoorStatus, WeighingStationModel
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid,Solid
from typing import Dict, List
from datetime import datetime


''' ==== Station Description ==== '''
class WeighingStation(Station):
    def __init__(self, station_model: WeighingStationModel) -> None:
        self._model = station_model
    
    @property
    def status(self) -> BalanceDoorStatus:
        self._model.reload('machine_status')
        return self._model.machine_status
    
    @status.setter
    def status(self, new_status: BalanceDoorStatus):
        self._model.update(machine_status=new_status)

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = WeighingStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, BalanceCloseDoorOpDescriptor):
            self.status = BalanceDoorStatus.DOORS_CLOSED
        elif isinstance(current_op, BalanceOpenDoorOpDescriptor):
            self.status = BalanceDoorStatus.DOORS_OPEN
        super().complete_assigned_station_op(success, **kwargs)

''' ==== Station Operation Descriptors ==== '''

class BalanceOpenDoorOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)


class BalanceCloseDoorOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)
    
class SampleWeighingOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: WeightOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = WeightOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def weight(self) -> int:
        return self._model.mass_reading

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'mass' in kwargs:
            self._model.mass = kwargs['mass']
        else:
            pass
    
    

    

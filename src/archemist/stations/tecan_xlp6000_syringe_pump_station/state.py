from .model import SyringePumpStatus, SyringePumpStationModel, SyringePumpOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid, Solid
from archemist.core.persistence.object_factory import MaterialFactory
from typing import Dict, List, Any
from archemist.core.exceptions.exception import InvalidLiquidError
from datetime import datetime


''' ==== Station Description ==== '''
class SyringePump(Station):
    def __init__(self, station_model: SyringePumpStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = SyringePumpStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def status(self) -> SyringePumpStatus:
        self._model.reload('machine_status')
        return self._model.machine_status

    @status.setter
    def status(self, new_status: SyringePumpStatus):
        self._model.update(machine_status=new_status)
    
    #to check if the station is idle 

    def assign_station_op(self, station_op: Any):
        if isinstance(station_op, SyringePumpDispenseOpDescriptor):
            self.status = SyringePumpStatus.DISPENSE
        elif isinstance(station_op, SyringePumpWithdrawOpDescriptor):
            self.status = SyringePumpStatus.WITHDRAW
        return super().assign_station_op(station_op)
    
    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, (SyringePumpDispenseOpDescriptor,SyringePumpWithdrawOpDescriptor)) and current_op.was_successful:
            self.status = SyringePumpStatus.JOB_COMPLETE
        super().complete_assigned_station_op(success, **kwargs)

''' ==== Station Operation Descriptors ==== '''
class SyringePumpDispenseOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: SyringePumpOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = SyringePumpOpDescriptorModel()
        model.port = kwargs["dispense_port"]
        model.speed = kwargs["dispense_speed"]
        model.volume = kwargs["dispense_volume"]
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def port(self) -> int:
        return self._model.port

    @property
    def dispense_speed(self) -> int:
        return self._model.speed

    @property
    def dispense_volume(self) -> int:
        return self._model.volume

class SyringePumpWithdrawOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: SyringePumpOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = SyringePumpOpDescriptorModel()
        model.port = int(kwargs["withdraw_port"])
        model.speed = float(kwargs["withdraw_speed"])
        model.volume = float(kwargs["withdraw_volume"])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def port(self) -> int:
        return self._model.port

    @property
    def withdraw_speed(self) -> int:
        return self._model.speed

    @property
    def withdraw_volume(self) -> int:
        return self._model.volume
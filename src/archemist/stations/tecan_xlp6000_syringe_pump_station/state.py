from .model import SyringePumpMode, SyringePumpStationModel, SyringePumpOpDescriptorModel
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
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = SyringePumpStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def mode(self) -> SyringePumpMode:
        self._model.reload('mode')
        return self._model.mode

    @mode.setter
    def mode(self, new_mode: SyringePumpMode):
        self._model.update(mode=new_mode)
    
    def assign_station_op(self, station_op: Any):
        if isinstance(station_op, SyringePumpDispenseOpDescriptor):
            self.mode = SyringePumpMode.DISPENSE
        elif isinstance(station_op, SyringePumpWithdrawOpDescriptor):
            self.mode = SyringePumpMode.WITHDRAW
        return super().assign_station_op(station_op)

''' ==== Station Operation Descriptors ==== '''
class SyringePumpDispenseOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: SyringePumpOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = SyringePumpOpDescriptorModel()
        model.port = int(kwargs['port'])
        model.speed = int(kwargs['dispense_speed'])
        model.volume = float(kwargs['dispense_volume'])
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
        model.port = int(kwargs['port'])
        model.speed = int(kwargs['withdraw_speed'])
        model.volume = float(kwargs['withdraw_volume'])
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
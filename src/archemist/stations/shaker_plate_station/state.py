from .model import ShakerStatus, ShakerPlateStationModel, ShakeOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from typing import List, Any, Dict
from archemist.core.state.material import Liquid, Solid

class ShakerPlateStation(Station):
    def __init__(self, station_model: ShakerPlateStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = ShakerPlateStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def status(self):
        self._model.reload('machine_status')
        return self._model.machine_status

    @status.setter
    def status(self, new_status: ShakerStatus):
        self._model.update(machine_status=new_status)

    def assign_station_op(self, stationOp: Any):
        if isinstance(stationOp, ShakeOpDescriptor):
            self.status = ShakerStatus.SHAKING
        super().assign_station_op(stationOp)

    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, ShakeOpDescriptor):
            self.status = ShakerStatus.NOT_SHAKING
        super().complete_assigned_station_op(success, **kwargs)

class ShakeOpDescriptor(StationOpDescriptor):
    def __init__(self, stationOpModel: ShakeOpDescriptorModel) -> None:
        self._model = stationOpModel

    @classmethod
    def from_args(cls, **kwargs):
        model = ShakeOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        model.duration = int(kwargs['duration'])
        return cls(model)

    @property
    def duration(self) -> int:
        return self._model.duration
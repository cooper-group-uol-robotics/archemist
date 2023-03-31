from .model import WatersLCMSStationModel, LCMSStatus, LCMSOpModel
from archemist.core.state.station import Station
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station_op import StationOpDescriptor
from typing import List, Any, Dict
from archemist.core.state.material import Liquid, Solid

''' ==== Station Description ==== '''
class WatersLCMSStation(Station):
    def __init__(self, station_model: WatersLCMSStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = WatersLCMSStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def status(self) -> LCMSStatus:
        self._model.reload('machine_status')
        return self._model.machine_status

    @status.setter
    def status(self, new_status: LCMSStatus):
        self._model.update(machine_status=new_status)

    def assign_station_op(self, stationOp: Any):
        if isinstance(stationOp, LCMSAnalysisOpDescriptor):
            self.status = LCMSStatus.RUNNING_ANALYSIS
        super().assign_station_op(stationOp)

    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, LCMSInsertBatchOpDescriptor):
            self.status = LCMSStatus.BATCH_LOADED
        elif isinstance(current_op, LCMSExtractBatchOpDescriptor):
            self.status = LCMSStatus.BATCH_READY_FOR_COLLECTION
        elif isinstance(current_op, LCMSAnalysisOpDescriptor): 
            self.status = LCMSStatus.ANALYSIS_COMPLETE
        super().complete_assigned_station_op(success, **kwargs)


''' ==== Station Operation Descriptors ==== '''
class LCMSInsertBatchOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: LCMSOpModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = LCMSOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        model.rack = int(kwargs['rack'])
        return cls(model)

    @property
    def rack(self) -> int:
        return self._model.rack

class LCMSExtractBatchOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: LCMSOpModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = LCMSOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        model.rack = kwargs['rack']
        return cls(model)

    @property
    def rack(self) -> int:
        return self._model.rack

class LCMSAnalysisOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)
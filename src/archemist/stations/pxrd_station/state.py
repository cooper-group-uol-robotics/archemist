from .model import PXRDStationModel, PXRDStatus, PXRDAnalysisOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from typing import List, Any, Dict
from archemist.core.state.material import Liquid, Solid
from datetime import datetime

''' ==== Station Description ==== '''
class PXRDStation(Station):
    def __init__(self, station_model: PXRDStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = PXRDStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def status(self) -> PXRDStatus:
        self._model.reload('machine_status')
        return self._model.machine_status

    @status.setter
    def status(self, new_status: PXRDStatus):
        self._model.update(machine_status=new_status)

    def assign_station_op(self, stationOp: Any):
        if isinstance(stationOp, PXRDAnalysisOpDescriptor):
            self.status = PXRDStatus.RUNNING_JOB
        super().assign_station_op(stationOp)

    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, PXRDAnalysisOpDescriptor) and current_op.was_successful:
            self.status = PXRDStatus.JOB_COMPLETE
        super().complete_assigned_station_op(success, **kwargs)

''' ==== Station Operation Descriptors ==== '''
class PXRDAnalysisOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: PXRDAnalysisOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = PXRDAnalysisOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def result_file(self) -> str:
        if self._model.has_result and self._model.was_successful:
            return self._model.result_file

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'result_file' in kwargs:
            self._model.result_file = kwargs['result_file']
        else:
            pass #print('missing result_file!!')
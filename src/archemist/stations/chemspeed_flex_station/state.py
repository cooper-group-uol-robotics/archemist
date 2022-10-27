from .model import ChemSpeedStatus, ChemSpeedFlexStationModel, CSCSVJobOpDescriptorModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from typing import List, Any, Dict
from archemist.core.state.material import Liquid, Solid
from datetime import datetime

''' ==== Station Description ==== '''
class ChemSpeedFlexStation(Station):
    def __init__(self, station_model: ChemSpeedFlexStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = ChemSpeedFlexStationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def status(self) -> ChemSpeedStatus:
        self._model.reload('machine_status')
        return self._model.machine_status

    @status.setter
    def status(self, new_status: ChemSpeedStatus):
        self._model.update(machine_status=new_status)

    def assign_station_op(self, stationOp: Any):
        if isinstance(stationOp, CSCSVJobOpDescriptor) or isinstance(stationOp, CSCSVJobOpDescriptor):
            self.status = ChemSpeedStatus.RUNNING_JOB
        super().assign_station_op(stationOp)

    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, CSOpenDoorOpDescriptor):
            self.status = ChemSpeedStatus.DOORS_OPEN
        elif isinstance(current_op, CSCloseDoorOpDescriptor):
            self.status = ChemSpeedStatus.DOORS_CLOSED
        elif (isinstance(current_op, CSCSVJobOpDescriptor) or 
                isinstance(current_op, CSCSVJobOpDescriptor) and 
                current_op.was_successful):
            self.status = ChemSpeedStatus.JOB_COMPLETE
        super().complete_assigned_station_op(success, **kwargs)


''' ==== Station Operation Descriptors ==== '''

class CSOpenDoorOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)


class CSCloseDoorOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class CSProcessingOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class CSCSVJobOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: CSCSVJobOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = CSCSVJobOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        model.csv_string = kwargs['csv_string']
        return cls(model)

    @property
    def csv_string(self) -> str:
        return self._model.csv_string

    @csv_string.setter
    def csv_string(self, new_csv_str: str):
        self._model.csv_string = new_csv_str

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
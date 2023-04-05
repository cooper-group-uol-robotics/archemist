from .model import ChemSpeedStatus, ChemSpeedFlexStationModel, CSCSVJobOpDescriptorModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.persistence.object_factory import MaterialFactory
from typing import List, Any, Dict
from archemist.core.state.material import Liquid, Solid
from datetime import datetime

""" ==== Station Description ==== """


class ChemSpeedFlexStation(Station):
    def __init__(self, station_model: ChemSpeedFlexStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = ChemSpeedFlexStationModel()
        cls._set_model_common_fields(station_dict, model)
        model._module = cls.__module__
        parameters = station_dict["parameters"]
        if parameters is not None:
            used_liquids = parameters["used_liquids"]
            for liquid_name in used_liquids:
                for liquid in liquids:
                    if liquid_name == liquid.name:
                        model.liquids_dict[liquid_name] = liquid.model.id
        model.save()
        return cls(model)

    @property
    def status(self) -> ChemSpeedStatus:
        self._model.reload("machine_status")
        return self._model.machine_status

    @property
    def used_liquids_names(self) -> List[str]:
        return list(self._model.liquids_dict.keys())

    @status.setter
    def status(self, new_status: ChemSpeedStatus):
        self._model.update(machine_status=new_status)

    def get_liquid(self, liquid_name: str) -> Liquid:
        liquid_obj_id = self._model.liquids_dict[liquid_name]
        return MaterialFactory.create_material_from_object_id(liquid_obj_id)

    def assign_station_op(self, station_op: Any):
        if isinstance(station_op, CSCSVJobOpDescriptor) or isinstance(
            station_op, CSCSVJobOpDescriptor
        ):
            self.status = ChemSpeedStatus.RUNNING_JOB
        super().assign_station_op(station_op)

    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, CSOpenDoorOpDescriptor):
            self.status = ChemSpeedStatus.DOORS_OPEN
        elif isinstance(current_op, CSCloseDoorOpDescriptor):
            self.status = ChemSpeedStatus.DOORS_CLOSED
        elif isinstance(current_op, CSCSVJobOpDescriptor):
            for liquid_name, dispense_vals in current_op.dispense_info.items():
                liquid_obj = self.get_liquid(liquid_name)
                liquid_obj.volume -= sum(dispense_vals) / 1000
            self.status = ChemSpeedStatus.JOB_COMPLETE
        elif isinstance(current_op, CSProcessingOpDescriptor):
            self.status = ChemSpeedStatus.JOB_COMPLETE
        super().complete_assigned_station_op(success, **kwargs)


""" ==== Station Operation Descriptors ==== """


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
        model.dispense_info = kwargs["dispense_info"]
        # FIXME check what key, value means for this dict
        for key, value in model.dispense_info.items():
            model.dispense_info[key] = list(map(float, value))
        return cls(model)

    @classmethod
    def clone_object(cls, obj):
        model = CSCSVJobOpDescriptorModel()
        model = obj._model
        return cls(model)

    @property
    def dispense_info(self):
        return self._model.dispense_info

    @dispense_info.setter
    def dispense_info(self, new_dispense_info):
        self._model.dispense_info = new_dispense_info

    @property
    def result_file(self) -> str:
        if self._model.has_result and self._model.was_successful:
            return self._model.result_file

    def get_csv_string(self) -> str:
        dispense_lists = list(self._model.dispense_info.values())
        zipped_tuples = zip(*list(dispense_lists))
        csv_string = ""
        for tup in zipped_tuples:
            tmp = ",".join(map(str, tup))
            csv_string += tmp + r"\n"
        return csv_string

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if "result_file" in kwargs:
            self._model.result_file = kwargs["result_file"]
        else:
            pass  # print('missing result_file!!')

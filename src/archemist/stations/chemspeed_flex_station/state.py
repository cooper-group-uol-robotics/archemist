from archemist.core.persistence.models_proxy import ModelProxy, DictProxy
from .model import ChemSpeedJobStatus, ChemSpeedFlexStationModel, CSLiquidDispenseOpModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.lot import Lot
from archemist.core.state.station_op import StationOpDescriptor, StationLotOpDescriptor
from archemist.core.state.station_op_result import StationOpResult
from typing import List, Dict, Type, Union
from archemist.core.util.enums import OpOutcome

''' ==== Station Description ==== '''
class ChemSpeedFlexStation(Station):
    def __init__(self, station_model: Union[ChemSpeedFlexStationModel,ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: dict):
        model = ChemSpeedFlexStationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)

    @property
    def job_status(self) -> ChemSpeedJobStatus:
        return self._model_proxy.job_status
    
    @job_status.setter
    def job_status(self, new_status: ChemSpeedJobStatus):
        self._model_proxy.job_status = new_status

    @property
    def door_closed(self) -> bool:
        return self._model_proxy.door_closed
    
    @door_closed.setter
    def door_closed(self, closed: bool):
        self._model_proxy.door_closed = closed

    def update_assigned_op(self):
        super().update_assigned_op()
        current_op = self.assigned_op
        if isinstance(current_op, CSLiquidDispenseOp) or isinstance(current_op, CSRunJobOp):
            self.job_status = ChemSpeedJobStatus.RUNNING_JOB

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        current_op = self.assigned_op
        if isinstance(current_op, CSOpenDoorOp):
            self.door_closed = False
        elif isinstance(current_op, CSCloseDoorOp):
            self.door_closed = True
        elif isinstance(current_op, CSLiquidDispenseOp):
            dispense_unit = current_op.dispense_unit
            for liq_name, liq_quantities in current_op.dispense_table.items():
                liquid = self.liquids_dict[liq_name]
                liq_total_quantity = sum(liq_quantities)
                liquid.decrease_volume(liq_total_quantity, dispense_unit)
            self.job_status = ChemSpeedJobStatus.JOB_COMPLETE
        elif isinstance(current_op, CSRunJobOp):
            self.job_status = ChemSpeedJobStatus.JOB_COMPLETE
        super().complete_assigned_op(outcome, results)


''' ==== Station Operation Descriptors ==== '''

class CSOpenDoorOp(StationOpDescriptor):
    def __init__(self, station_op_model: Union[StationOpDescriptorModel,ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=ChemSpeedFlexStation.__name__)
        model.save()
        return cls(model)


class CSCloseDoorOp(StationOpDescriptor):
    def __init__(self, station_op_model: Union[StationOpDescriptorModel,ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=ChemSpeedFlexStation.__name__)
        model.save()
        return cls(model)

class CSRunJobOp(StationOpDescriptor):
    def __init__(self, station_op_model: Union[StationOpDescriptorModel,ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=ChemSpeedFlexStation.__name__)
        model.save()
        return cls(model)

class CSLiquidDispenseOp(StationLotOpDescriptor):
    def __init__(self, station_op_model: Union[CSLiquidDispenseOpModel,ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls, target_lot: Lot,
                  dispense_table: Dict[str, List[float]],
                  dispense_unit: str):
        model = CSLiquidDispenseOpModel()
        model.target_lot = target_lot.model
        cls._set_model_common_fields(model, associated_station=ChemSpeedFlexStation.__name__)
        model.dispense_table = {key: map(float, v_list) for key, v_list in dispense_table.items()}
        model.dispense_unit = dispense_unit
        model.save()
        return cls(model)
    
    @property
    def dispense_table(self) -> Dict[str, List[float]]:
        return self._model_proxy.dispense_table
    
    @property
    def dispense_unit(self) -> str:
        return self._model_proxy.dispense_unit

    def to_csv_string(self) -> str:
        qtys_lists = [qtys for qtys in self.dispense_table.values()]
        zipped_qtys_tuples = zip(*[qtys for qtys in qtys_lists])
        csv_string = ""
        for qtys_tup in zipped_qtys_tuples:
            row = ','.join(map(str,qtys_tup))
            csv_string += row + r"\n"
        return csv_string
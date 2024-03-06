from .model import ShakerPlateStationModel, ShakerPlateOpModel
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station
from archemist.core.state.batch import Batch
from archemist.core.state.station_op import StationBatchOp
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.util.enums import OpOutcome

from typing import List, Union, Dict, Type, Literal


class ShakerPlateStation(Station):
    def __init__(self, station_model: Union[ShakerPlateStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = ShakerPlateStationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)

    @property
    def is_shaking(self) -> bool:
        return self._model_proxy.is_shaking

    @is_shaking.setter
    def is_shaking(self, shaking: bool):
        self._model_proxy.is_shaking = shaking

    def update_assigned_op(self):
        super().update_assigned_op()
        current_op = self.assigned_op
        if isinstance(current_op, ShakerPlateOp):
            self.is_shaking = True

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        current_op = self.assigned_op
        if isinstance(current_op, ShakerPlateOp):
            self.is_shaking = False
        super().complete_assigned_op(outcome, results)


class ShakerPlateOp(StationBatchOp):
    def __init__(self, station_op_model: Union[ShakerPlateOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls,
                  target_batch: Batch,
                  duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = ShakerPlateOpModel()
        cls._set_model_common_fields(
            model, associated_station=ShakerPlateStation.__name__)
        model.target_batch = target_batch.model
        model.duration = duration
        model.time_unit = time_unit
        model.save()
        return cls(model)

    @property
    def duration(self) -> int:
        return self._model_proxy.duration

    @property
    def time_unit(self) -> Literal["second", "minute", "hour"]:
        return self._model_proxy.time_unit

from .model import WaitOpModel
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station
from archemist.core.state.lot import Lot
from archemist.core.models.station_model import StationModel
from archemist.core.state.station_op import StationLotOp
from typing import Dict, Union, Literal


''' ==== Station Description ==== '''


class WaitingStation(Station):
    def __init__(self, station_model: Union[StationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = StationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)


''' ==== Station Operation Descriptors ==== '''


class WaitOp(StationLotOp):
    def __init__(self, station_op_model: Union[WaitOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls,
                  target_lot: Lot,
                  duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = WaitOpModel()
        model.target_lot = target_lot.model
        cls._set_model_common_fields(
            model, associated_station=WaitingStation.__name__)
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

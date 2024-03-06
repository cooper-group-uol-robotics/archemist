from typing import Dict, Union, Literal
from archemist.core.persistence.models_proxy import ModelProxy
from .model import APCDryProductOpModel
from archemist.core.state.station import Station, StationModel
from archemist.core.state.station_op import StationOp, StationSampleOp, StationOpModel
from archemist.core.state.sample import Sample


''' ==== Station Description ==== '''


class APCFiltrationStation(Station):
    def __init__(self, filtration_station_model: Union[StationModel, ModelProxy]) -> None:
        super().__init__(filtration_station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = StationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)


''' ==== Station Operation Descriptors ==== '''


class APCFilterProductOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCFiltrationStation.__name__)
        model.save()
        return cls(model)


class APCDryProductOp(StationSampleOp):
    def __init__(self, op_model: Union[APCDryProductOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = APCDryProductOpModel()
        cls._set_model_common_fields(model, associated_station=APCFiltrationStation.__name__)
        model.target_sample = target_sample.model
        model.duration = int(duration)
        model.time_unit = time_unit
        model.save()
        return cls(model)

    @property
    def duration(self) -> int:
        return self._model_proxy.duration

    @property
    def time_unit(self) -> Literal["second", "minute", "hour"]:
        return self._model_proxy.time_unit


class APCDrainWasteOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCFiltrationStation.__name__)
        model.save()
        return cls(model)

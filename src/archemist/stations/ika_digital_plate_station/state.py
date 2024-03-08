from .model import IKADigitalPlateMode, IkaDigitalPlateStationModel, IKADigitalPlateOpModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationBatchOp, StationOp, StationOpModel
from archemist.core.state.batch import Batch
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station_op_result import ProcessOpResult
from archemist.core.util.enums import OpOutcome
from typing import Literal, Dict, Union, List


''' ==== Station Description ==== '''


class IKADigitalPlateStation(Station):
    def __init__(self, station_model: Union[IkaDigitalPlateStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = IkaDigitalPlateStationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)

    @property
    def current_temperature(self) -> int:
        return self._model_proxy.current_temperature

    @current_temperature.setter
    def current_temperature(self, new_temp: int):
        self._model_proxy.current_temperature = new_temp

    @property
    def current_stirring_speed(self) -> int:
        return self._model_proxy.current_stirring_speed

    @current_stirring_speed.setter
    def current_stirring_speed(self, new_speed: int):
        self._model_proxy.current_stirring_speed = new_speed

    @property
    def external_temperature(self) -> int:
        return self._model_proxy.external_temperature

    @external_temperature.setter
    def external_temperature(self, new_temp: int):
        self._model_proxy.external_temperature = new_temp

    @property
    def viscosity_trend(self) -> float:
        return self._model_proxy.viscosity_trend

    @viscosity_trend.setter
    def viscosity_trend(self, value: float):
        self._model_proxy.viscosity_trend = value

    @property
    def mode(self) -> IKADigitalPlateMode:
        return self._model_proxy.mode

    @mode.setter
    def mode(self, new_mode: IKADigitalPlateMode):
        self._model_proxy.mode = new_mode

    def update_assigned_op(self):
        super().update_assigned_op()
        current_op = self.assigned_op
        if isinstance(current_op, IKAHeatStirBatchOp):
            self.mode = IKADigitalPlateMode.HEATING_STIRRING
        elif isinstance(current_op, IKAHeatBatchOp):
            self.mode = IKADigitalPlateMode.HEATING
        elif isinstance(current_op, IKAStirBatchOp):
            self.mode = IKADigitalPlateMode.STIRRING

    def complete_assigned_op(self, outcome: OpOutcome, results: List[ProcessOpResult]):
        current_op = self.assigned_op
        if isinstance(current_op, IKAStopOp):
            self.mode = None
        elif current_op.duration > 0:
            self.mode = None
        super().complete_assigned_op(outcome, results)


''' ==== Station Operation Descriptors ==== '''


class IKAHeatStirBatchOp(StationBatchOp):
    def __init__(self, op_model: Union[IKADigitalPlateOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_batch: Batch,
                  target_temperature: int,
                  target_stirring_speed: int,
                  duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = IKADigitalPlateOpModel()
        model.target_batch = target_batch.model
        cls._set_model_common_fields(
            model, associated_station=IKADigitalPlateStation.__name__)
        model.target_temperature = int(target_temperature)
        model.target_stirring_speed = int(target_stirring_speed)
        model.duration = int(duration)
        if model.duration > 0:
            model.time_unit = time_unit
        model.save()
        return cls(model)

    @property
    def target_temperature(self) -> int:
        return self._model_proxy.target_temperature

    @property
    def target_stirring_speed(self) -> int:
        return self._model_proxy.target_stirring_speed

    @property
    def duration(self) -> int:
        return self._model_proxy.duration

    @property
    def time_unit(self) -> Literal["second", "minute", "hour"]:
        return self._model_proxy.time_unit


class IKAHeatBatchOp(StationBatchOp):
    def __init__(self, op_model: Union[IKADigitalPlateOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_batch: Batch,
                  target_temperature: int,
                  duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = IKADigitalPlateOpModel()
        model.target_batch = target_batch.model
        cls._set_model_common_fields(
            model, associated_station=IKADigitalPlateStation.__name__)
        model.target_temperature = int(target_temperature)
        model.duration = int(duration)
        if model.duration > 0:
            model.time_unit = time_unit
        model.save()
        return cls(model)

    @property
    def target_temperature(self) -> int:
        return self._model_proxy.target_temperature

    @property
    def duration(self) -> int:
        return self._model_proxy.duration

    @property
    def time_unit(self) -> Literal["second", "minute", "hour"]:
        return self._model_proxy.time_unit


class IKAStirBatchOp(StationBatchOp):
    def __init__(self, op_model: Union[IKADigitalPlateOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_batch: Batch,
                  target_stirring_speed: int,
                  duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = IKADigitalPlateOpModel()
        model.target_batch = target_batch.model
        cls._set_model_common_fields(
            model, associated_station=IKADigitalPlateStation.__name__)
        model.target_stirring_speed = int(target_stirring_speed)
        model.duration = int(duration)
        if model.duration > 0:
            model.time_unit = time_unit
        model.save()
        return cls(model)

    @property
    def target_stirring_speed(self) -> int:
        return self._model_proxy.target_stirring_speed

    @property
    def duration(self) -> int:
        return self._model_proxy.duration

    @property
    def time_unit(self) -> Literal["second", "minute", "hour"]:
        return self._model_proxy.time_unit


class IKAStopOp(StationOp):
    def __init__(self, station_op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(
            model, associated_station=IKADigitalPlateStation.__name__)
        model.save()
        return cls(model)

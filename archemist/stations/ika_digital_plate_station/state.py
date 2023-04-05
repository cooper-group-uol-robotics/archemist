from .model import IKAMode, IkaPlateDigitalModel, IKAOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from typing import Any, Dict


class IkaPlateDigital(Station):
    """ ==== Station Description ==== """
    def __init__(self, station_model: IkaPlateDigitalModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = IkaPlateDigitalModel()
        cls._set_model_common_fields(station_dict, model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def current_temperature(self) -> int:
        self._model.reload("current_temperature")
        return self._model.current_temperature

    @current_temperature.setter
    def current_temperature(self, new_temp: int):
        self._model.update(current_temperature=new_temp)

    @property
    def target_temperature(self) -> int:
        self._model.reload("target_temperature")
        return self._model.target_temperature

    @target_temperature.setter
    def target_temperature(self, new_temp: int):
        self._model.update(target_temperature=new_temp)

    @property
    def current_stirring_speed(self) -> int:
        self._model.reload("current_stirring_speed")
        return self._model.current_stirring_speed

    @current_stirring_speed.setter
    def current_stirring_speed(self, new_speed: int):
        self._model.update(current_stirring_speed=new_speed)

    @property
    def target_stirring_speed(self) -> int:
        self._model.reload("target_stirring_speed")
        return self._model.target_stirring_speed

    @target_stirring_speed.setter
    def target_stirring_speed(self, new_speed: int):
        self._model.update(target_stirring_speed=new_speed)

    @property
    def target_duration(self) -> int:
        self._model.reload("target_duration")
        return self._model.target_duration

    @target_duration.setter
    def target_duration(self, new_duration: int):
        self._model.update(target_duration=new_duration)

    @property
    def external_temperature(self) -> int:
        self._model.reload("external_temperature")
        return self._model.external_temperature

    @external_temperature.setter
    def external_temperature(self, new_temp: int):
        self._model.update(external_temperature=new_temp)

    @property
    def viscosity_trend(self) -> float:
        self._model.reload("viscosity_trend")
        return self._model.viscosity_trend

    @viscosity_trend.setter
    def viscosity_trend(self, value: float):
        self._model.update(viscosity_trend=value)

    @property
    def mode(self) -> IKAMode:
        self._model.reload("mode")
        return self._model.mode

    @mode.setter
    def mode(self, new_mode: IKAMode):
        self._model.update(mode=new_mode)

    def assign_station_op(self, station_op: Any):
        if isinstance(station_op, IKAHeatingStirringOpDescriptor):
            self.target_temperature = station_op.target_temperature
            self.target_stirring_speed = station_op.target_stirring_speed
            self.target_duration = station_op.target_duration
            self.mode = IKAMode.HEATINGSTIRRING
        elif isinstance(station_op, IKAHeatingOpDescriptor):
            self.target_temperature = station_op.target_temperature
            self.target_duration = station_op.target_duration
            self.mode = IKAMode.HEATING
        elif isinstance(station_op, IKAStirringOpDescriptor):
            self.target_stirring_speed = station_op.target_stirring_speed
            self.target_duration = station_op.target_duration
            self.mode = IKAMode.STIRRING
        super().assign_station_op(station_op)

    def complete_assigned_station_op(self, success: bool, **kwargs):
        self._model.update(unset__target_temperature=True)
        self._model.update(unset__target_stirring_speed=True)
        self._model.update(unset__target_duration=True)
        self._model.update(unset__mode=True)
        super().complete_assigned_station_op(success, **kwargs)


class IKAHeatingStirringOpDescriptor(StationOpDescriptor):
    """ ==== Station Operation Descriptors ==== """
    def __init__(self, op_model: IKAOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = IKAOpDescriptorModel()
        model.target_temperature = int(kwargs["temperature"])
        model.target_stirring_speed = int(kwargs["stirring_speed"])
        model.target_duration = float(kwargs["duration"])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def target_temperature(self) -> int:
        return self._model.target_temperature

    @property
    def target_stirring_speed(self) -> int:
        return self._model.target_stirring_speed

    @property
    def target_duration(self) -> int:
        return self._model.target_duration


class IKAHeatingOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: IKAOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = IKAOpDescriptorModel()
        model.target_temperature = int(kwargs["temperature"])
        model.target_duration = float(kwargs["duration"])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def target_temperature(self) -> int:
        return self._model.target_temperature

    @property
    def target_duration(self) -> int:
        return self._model.target_duration


class IKAStirringOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: IKAOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = IKAOpDescriptorModel()
        model.target_stirring_speed = int(kwargs["stirring_speed"])
        model.target_duration = float(kwargs["duration"])
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def target_stirring_speed(self) -> int:
        return self._model.target_stirring_speed

    @property
    def target_duration(self) -> int:
        return self._model.target_duration

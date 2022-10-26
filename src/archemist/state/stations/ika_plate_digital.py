from archemist.state.station import Station, StationModel, StationOpDescriptorModel, StationOpDescriptor
from enum import Enum
from typing import List, Any, Dict
from archemist.state.material import Liquid, Solid
from mongoengine import fields

class IKAMode(Enum):
    HEATING = 1
    STIRRING = 2
    HEATINGSTIRRING = 3

''' ==== Station Description ==== '''

class IkaPlateDigitalModel(StationModel):
    mode = fields.EnumField(IKAMode, null=True)
    current_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    target_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    current_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)
    external_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    viscosity_trend = fields.FloatField(null=True)
    target_duration = fields.FloatField(null=True)


class IkaPlateDigital(Station):
    def __init__(self, station_model: IkaPlateDigitalModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = IkaPlateDigitalModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

    @property
    def current_temperature(self) -> int:
        self._model.reload('current_temperature')
        return self._model.current_temperature

    @current_temperature.setter
    def current_temperature(self, new_temp: int):
        self._model.update(current_temperature=new_temp)

    @property
    def target_temperature(self) -> int:
        self._model.reload('target_temperature')
        return self._model.target_temperature

    @target_temperature.setter
    def target_temperature(self, new_temp: int):
        self._model.update(target_temperature=new_temp)

    @property
    def current_stirring_speed(self) -> int:
        self._model.reload('current_stirring_speed')
        return self._model.current_stirring_speed

    @current_stirring_speed.setter
    def current_stirring_speed(self, new_speed: int):
        self._model.update(current_stirring_speed=new_speed)

    @property
    def target_stirring_speed(self) -> int:
        self._model.reload('target_stirring_speed')
        return self._model.target_stirring_speed

    @target_stirring_speed.setter
    def target_stirring_speed(self, new_speed: int):
        self._model.update(target_stirring_speed=new_speed)

    @property
    def target_duration(self) -> int:
        self._model.reload('target_duration')
        return self._model.target_duration

    @target_duration.setter
    def target_duration(self, new_duration: int):
        self._model.update(target_duration=new_duration)

    @property
    def external_temperature(self) -> int:
        self._model.reload('external_temperature')
        return self._model.external_temperature

    @external_temperature.setter
    def external_temperature(self, new_temp: int):
        self._model.update(external_temperature=new_temp)

    @property
    def viscosity_trend(self) -> float:
        self._model.reload('viscosity_trend')
        return self._model.viscosity_trend

    @viscosity_trend.setter
    def viscosity_trend(self, value: float):
        self._model.update(viscosity_trend=value)

    @property
    def mode(self) -> IKAMode:
        self._model.reload('mode')
        return self._model.mode

    @mode.setter
    def mode(self, new_mode: IKAMode):
        self._model.update(mode=new_mode)

    def assign_station_op(self, stationOp: Any):
        if isinstance(stationOp, IKAHeatingStirringOpDescriptor):
            self.target_temperature = stationOp.target_temperature
            self.target_stirring_speed = stationOp.target_stirring_speed
            self.target_duration = stationOp.target_duration
            self.mode = IKAMode.HEATINGSTIRRING
        elif isinstance(stationOp, IKAHeatingOpDescriptor):
            self.target_temperature = stationOp.target_temperature
            self.target_duration = stationOp.target_duration
            self.mode = IKAMode.HEATING
        elif isinstance(stationOp, IKAStirringOpDescriptor):
            self.target_stirring_speed = stationOp.target_stirring_speed
            self.target_duration = stationOp.target_duration
            self.mode = IKAMode.STIRRING
        super().assign_station_op(stationOp)

    def complete_assigned_station_op(self, success: bool, **kwargs):
        self._model.update(unset__target_temperature=True)
        self._model.update(unset__target_stirring_speed=True)
        self._model.update(unset__target_duration=True)
        self._model.update(unset__mode=True)
        super().complete_assigned_station_op(success, **kwargs)


''' ==== Station Operation Descriptors ==== '''

class IKAOpDescriptorModel(StationOpDescriptorModel):
    target_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)
    target_duration = fields.FloatField(null=True)

class IKAHeatingStirringOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: IKAOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = IKAOpDescriptorModel()
        model.target_temperature = kwargs['temperature']
        model.target_stirring_speed = kwargs['stirring_speed']
        model.target_duration = kwargs['duration']
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
        model.target_temperature = kwargs['temperature']
        model.target_duration = kwargs['duration']
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
        model.target_stirring_speed = kwargs['stirring_speed']
        model.target_duration = kwargs['duration']
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def target_stirring_speed(self) -> int:
        return self._model.target_stirring_speed

    @property
    def target_duration(self) -> int:
        return self._model.target_duration

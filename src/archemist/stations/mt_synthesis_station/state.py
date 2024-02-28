from typing import Dict, Union, Any, List, Literal, Type
from archemist.core.persistence.models_proxy import ModelProxy, EmbedModelProxy, ListProxy
from .model import (MTSynthesisStationModel,
                    OptiMaxMode,
                    MTSynthHeatStirOpModel,
                    MTSynthSampleOpModel,
                    MTSynthCustomOpenCloseReactionValveOpModel)
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOp, StationSampleOp, StationOpModel
from archemist.core.state.sample import Sample
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.util.enums import OpOutcome

class MTSynthesisStation(Station):
    def __init__(self, station_model: Union[MTSynthesisStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = MTSynthesisStationModel()
        cls._set_model_common_fields(model, station_dict)
        model.num_sampling_vials = station_dict['properties']['num_sampling_vials']
        model.save()
        return cls(model)

    @property
    def optimax_mode(self) -> OptiMaxMode:
        return self._model_proxy.optimax_mode

    @optimax_mode.setter
    def optimax_mode(self, new_mode: OptiMaxMode):
        self._model_proxy.optimax_mode = new_mode

    @property
    def optimax_valve_open(self) -> bool:
        return self._model_proxy.optimax_valve_open

    @optimax_valve_open.setter
    def optimax_valve_open(self, new_state: bool):
        self._model_proxy.optimax_valve_open = new_state

    @property
    def num_sampling_vials(self) -> int:
        return self._model_proxy.num_sampling_vials

    @num_sampling_vials.setter
    def num_sampling_vials(self, new_value: int):
        self._model_proxy.num_sampling_vials = new_value

    @property
    def set_reaction_temperature(self) -> int:
        return self._model_proxy.set_reaction_temperature

    @set_reaction_temperature.setter
    def set_reaction_temperature(self, new_value: int):
        self._model_proxy.set_reaction_temperature = new_value

    @property
    def set_stirring_speed(self) -> int:
        return self._model_proxy.set_stirring_speed

    @set_stirring_speed.setter
    def set_stirring_speed(self, new_value: int):
        self._model_proxy.set_stirring_speed = new_value

    def update_assigned_op(self):
        super().update_assigned_op()
        op = self.assigned_op
        if isinstance(op, MTSynthHeatStirOp) or isinstance(op, MTSynthSampleOp):
            if op.target_temperature is not None and op.target_stirring_speed is not None:
                self.optimax_mode = OptiMaxMode.HEATING_STIRRING
                self.set_reaction_temperature = op.target_temperature
                self.set_stirring_speed = op.target_stirring_speed
            elif op.target_temperature is not None:
                self.optimax_mode = OptiMaxMode.HEATING
                self.set_reaction_temperature = op.target_temperature
                self.set_stirring_speed = None
            elif op.target_stirring_speed is not None:
                self.optimax_mode = OptiMaxMode.STIRRING
                self.set_reaction_temperature = None
                self.set_stirring_speed = op.target_stirring_speed
        elif isinstance(op, MTSynthCustomOpenCloseReactionValveOp):
            self.optimax_valve_open = True

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        op = self.assigned_op
        if isinstance(op, MTSynthHeatStirOp) or isinstance(op, MTSynthStopReactionOp):
            self.optimax_mode =None
        elif isinstance(op, MTSynthSampleOp):
            self.num_sampling_vials -= 1
        elif isinstance(op, MTSynthOpenReactionValveOp):
            self.optimax_valve_open = True
        elif isinstance(op, MTSynthCloseReactionValveOp) or isinstance(op, MTSynthCustomOpenClosrReactionValveOp):
            self.optimax_valve_open = False
        super().complete_assigned_op(outcome, results)



''' ==== Station Operation Descriptors ==== '''

class MTSynthHeatStirOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthHeatStirOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  target_temperature: int,
                  target_stirring_speed: int,
                  wait_duration: int,
                  stir_duration:int,
                  time_unit: Literal["second", "minute", "hour"]=None):
        model = MTSynthHeatStirOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_temperature = int(target_temperature) if target_temperature else None
        model.target_stirring_speed = int(target_stirring_speed) if target_stirring_speed else None
        model.wait_duration = int(wait_duration) if wait_duration and wait_duration != "Null" else None
        model.stir_duration = int(stir_duration) if stir_duration and stir_duration != "Null" else None
        if model.wait_duration:
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
    def stir_duration(self) -> int:
        return self._model_proxy.stir_duration

    @property
    def time_unit(self) -> Literal["second", "minute", "hour"]:
        return self._model_proxy.time_unit
  
    @property
    def wait_duration(self) -> int:
        return self._model_proxy.wait_duration

class MTSynthSampleOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthSampleOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  target_temperature: int,
                  target_stirring_speed: int,
                  dilution: int):
        model = MTSynthSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_temperature = int(target_temperature) if target_temperature else None
        model.target_stirring_speed = int(target_stirring_speed) if target_stirring_speed else None
        model.dilution = int(dilution) if dilution else None
        model.save()
        return cls(model)

    @property
    def target_temperature(self) -> int:
        return self._model_proxy.target_temperature

    @property
    def target_stirring_speed(self) -> int:
        return self._model_proxy.target_stirring_speed
    
    @property
    def dilution(self) -> int:
        return self._model_proxy.dilution

class MTSynthStopReactionOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)
    
class MTSynthCustomOpenCloseReactionValveOp(StationOp):
    def __init__(self, op_model: Union[MTSynthCustomOpenCloseReactionValveOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, steps: int):
        model = MTSynthCustomOpenCloseReactionValveOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.steps = int(steps)
        model.save()
        return cls(model)
    
    @property
    def steps(self) -> int:
        return self._model_proxy.steps

class MTSynthOpenReactionValveOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

class MTSynthCloseReactionValveOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

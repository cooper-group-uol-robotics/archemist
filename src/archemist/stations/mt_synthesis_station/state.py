from typing import Dict, Union, Any, List, Literal, Type
from archemist.core.persistence.models_proxy import ModelProxy, EmbedModelProxy, ListProxy
from .model import (MTSynthesisStationModel,
                    SynthesisCartridgeModel,
                    OptiMaxMode,
                    MTSynthDispenseSolidOpModel,
                    MTSynthDispenseLiquidOpModel,
                    MTSynthStartLiquidDispensingOpModel,
                    MTSynthStopLiquidDispensingOpModel,
                    MTSynthHeatStirOpModel,
                    MTSynthSampleOpModel,
                    MTSynthAddWashLiquidOpModel,
                    MTSynthTimedOpenReactionValveOpModel,
                    MTSynthDryOpModel)
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOp, StationSampleOp, StationOpModel
from archemist.core.state.sample import Sample
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.util.enums import OpOutcome

class SynthesisCartridge:
    def __init__(self, cartridge_model: Union[SynthesisCartridgeModel, EmbedModelProxy]):
        self._model_proxy = cartridge_model

    @classmethod
    def from_dict(cls, cartridge_dict: Dict[str, Any]):
        model = SynthesisCartridgeModel()
        model.associated_solid = cartridge_dict["associated_solid"]
        model.hotel_index = cartridge_dict["hotel_index"]
        return cls(model)

    @property
    def model(self) -> SynthesisCartridgeModel:
        if isinstance(self._model_proxy, EmbedModelProxy):
            return self._model_proxy.model
        else:
            return self._model_proxy

    @property
    def associated_solid(self) -> str:
        return self._model_proxy.associated_solid

    @property
    def hotel_index(self) -> int:
        return self._model_proxy.hotel_index

    @property
    def depleted(self) -> bool:
        return self._model_proxy.depleted

    @depleted.setter
    def depleted(self, is_depleted: bool):
        self._model_proxy.depleted = is_depleted

class MTSynthesisStation(Station):
    def __init__(self, station_model: Union[MTSynthesisStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = MTSynthesisStationModel()
        cls._set_model_common_fields(model, station_dict)
        for cartridge_dict in station_dict['properties']['cartridges']:
            cartridge = SynthesisCartridge.from_dict(cartridge_dict)
            model.cartridges.append(cartridge.model)
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
    def cartridges(self) -> List[SynthesisCartridge]:
        return ListProxy(self._model_proxy.cartridges, SynthesisCartridge)

    @property
    def loaded_cartridge(self) -> SynthesisCartridge:
        cartridge_index = self._model_proxy.loaded_cartridge_index
        if cartridge_index is not None:
            return self.cartridges[cartridge_index]

    @property
    def window_open(self) -> bool:
        return self._model_proxy.window_open

    @window_open.setter
    def window_open(self, new_state: bool):
        self._model_proxy.window_open = new_state

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
        elif isinstance(op, MTSynthTimedOpenReactionValveOp):
            self.optimax_valve_open = True

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        op = self.assigned_op
        if isinstance(op, MTSynthDispenseSolidOp):
            solid = self.solids_dict[op.solid_name]
            solid.decrease_mass(op.dispense_mass, op.dispense_unit)
            self.loaded_cartridge.depleted = True
        elif isinstance(op, MTSynthDispenseLiquidOp) or isinstance(op, MTSynthAddWashLiquidOp):
            liquid = self.liquids_dict[op.liquid_name]
            liquid.decrease_volume(op.dispense_volume, op.dispense_unit)
        elif isinstance(op, MTSynthStopLiquidDispensingOp):
            op_result = results[0]
            op.dispensed_volume = op_result.amounts[0]
            op.dispense_unit = op_result.units[0]
            liquid = self.liquids_dict[op.liquid_name]
            liquid.decrease_volume(op.dispensed_volume, op.dispense_unit)
        elif isinstance(op, MTSynthHeatStirOp) or isinstance(op, MTSynthStopReactionOp):
            self.optimax_mode =None
        elif isinstance(op, MTSynthSampleOp):
            self.num_sampling_vials -= 1
        elif isinstance(op, MTSynthOpenReactionValveOp):
            self.optimax_valve_open = True
        elif isinstance(op, MTSynthCloseReactionValveOp) or isinstance(op, MTSynthTimedOpenReactionValveOp):
            self.optimax_valve_open = False
        super().complete_assigned_op(outcome, results)

    def load_cartridge(self, cartridge_index: int):
        self._model_proxy.loaded_cartridge_index = cartridge_index

    def unload_cartridge(self):
        self._model_proxy.loaded_cartridge_index = None



''' ==== Station Operation Descriptors ==== '''
''' SOLID ADDITION OPERATIONS '''

class MTSynthDispenseSolidOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthDispenseSolidOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  solid_name: str,
                  dispense_mass: float,
                  dispense_unit: Literal["g", "mg", "ug"]):
        model = MTSynthDispenseSolidOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_sample = target_sample.model
        model.solid_name = solid_name
        model.dispense_mass = float(dispense_mass)
        model.dispense_unit = dispense_unit
        model.save()
        return cls(model)

    @property
    def solid_name(self) -> str:
        return self._model_proxy.solid_name

    @property
    def dispense_mass(self) -> float:
        return self._model_proxy.dispense_mass

    @property
    def dispense_unit(self) -> Literal["g", "mg", "ug"]:
        return self._model_proxy.dispense_unit

''' LIQUID ADDITION OPERATIONS '''
class MTSynthDispenseLiquidOp(StationSampleOp):
    def __init__(self, station_op_model: Union[MTSynthDispenseLiquidOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)


    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  liquid_name: str, 
                  dispense_volume: float,
                  dispense_unit: Literal["L", "mL", "uL"]):
        model = MTSynthDispenseLiquidOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_sample = target_sample.model
        model.liquid_name = liquid_name
        model.dispense_volume = float(dispense_volume)
        model.dispense_unit = dispense_unit
        model.save()
        return cls(model)
        

    @property
    def liquid_name(self) -> str:
        return self._model_proxy.liquid_name

    @property
    def dispense_volume(self) -> float:
        return self._model_proxy.dispense_volume

    @property
    def dispense_unit(self) -> Literal["L", "mL", "uL"]:
        return self._model_proxy.dispense_unit

class MTSynthStartLiquidDispensingOp(StationOp):
    def __init__(self, station_op_model: Union[MTSynthStartLiquidDispensingOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)


    @classmethod
    def from_args(cls,
                  liquid_name: str,
                  dispense_rate: float,
                  rate_unit: Literal["mL/minute", "mL/second"],
                  max_dispense_volume: float,
                  dispense_unit: Literal["L", "mL", "uL"]):
        model = MTSynthStartLiquidDispensingOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.liquid_name = liquid_name
        model.dispense_rate = float(dispense_rate)
        model.rate_unit = rate_unit
        model.max_dispense_volume = float(max_dispense_volume)
        model.dispense_unit = dispense_unit
        model.save()
        return cls(model)
        

    @property
    def liquid_name(self) -> str:
        return self._model_proxy.liquid_name

    @property
    def dispense_rate(self) -> float:
        return self._model_proxy.dispense_rate

    @property
    def rate_unit(self) -> Literal["mL/minute", "mL/second"]:
        return self._model_proxy.rate_unit

    @property
    def max_dispense_volume(self) -> float:
        return self._model_proxy.max_dispense_volume

    @property
    def dispense_unit(self) -> Literal["L", "mL", "uL"]:
        return self._model_proxy.dispense_unit

class MTSynthStopLiquidDispensingOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthStopLiquidDispensingOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample,
                  liquid_name: str):
        model = MTSynthStopLiquidDispensingOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_sample = target_sample.model
        model.liquid_name = liquid_name
        model.save()
        return cls(model)

    @property
    def liquid_name(self) -> str:
        return self._model_proxy.liquid_name

    @property
    def dispensed_volume(self) -> float:
        return self._model_proxy.dispensed_volume

    @dispensed_volume.setter
    def dispensed_volume(self, new_value: float):
        self._model_proxy.dispensed_volume = new_value

    @property
    def dispense_unit(self) -> Literal["L", "mL", "uL"]:
        return self._model_proxy.dispense_unit

    @dispense_unit.setter
    def dispense_unit(self, unit: Literal["L", "mL", "uL"]):
        self._model_proxy.dispense_unit = unit

class MTSynthAddWashLiquidOp(StationOp):
    def __init__(self, station_op_model: Union[MTSynthAddWashLiquidOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)


    @classmethod
    def from_args(cls,
                  liquid_name: str, 
                  dispense_volume: float,
                  dispense_unit: Literal["L", "mL", "uL"]):
        model = MTSynthAddWashLiquidOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.liquid_name = liquid_name
        model.dispense_volume = float(dispense_volume)
        model.dispense_unit = dispense_unit
        model.save()
        return cls(model)

    @property
    def liquid_name(self) -> str:
        return self._model_proxy.liquid_name

    @property
    def dispense_volume(self) -> float:
        return self._model_proxy.dispense_volume

    @property
    def dispense_unit(self) -> Literal["L", "mL", "uL"]:
        return self._model_proxy.dispense_unit

''' OPTIMAX OPERATIONS '''
class MTSynthHeatStirOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthHeatStirOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  target_temperature: int,
                  target_stirring_speed: int,
                  wait_duration: int,
                  time_unit: Literal["second", "minute", "hour"]=None):
        model = MTSynthHeatStirOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_temperature = int(target_temperature) if target_temperature else None
        model.target_stirring_speed = int(target_stirring_speed) if target_stirring_speed else None
        model.wait_duration = int(wait_duration) if wait_duration and wait_duration != "Null" else None
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
    def wait_duration(self) -> int:
        return self._model_proxy.wait_duration

    @property
    def time_unit(self) -> Literal["second", "minute", "hour"]:
        return self._model_proxy.time_unit

class MTSynthSampleOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthSampleOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  target_temperature: int,
                  target_stirring_speed: int):
        model = MTSynthSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_temperature = int(target_temperature) if target_temperature else None
        model.target_stirring_speed = int(target_stirring_speed) if target_stirring_speed else None
        model.save()
        return cls(model)

    @property
    def target_temperature(self) -> int:
        return self._model_proxy.target_temperature

    @property
    def target_stirring_speed(self) -> int:
        return self._model_proxy.target_stirring_speed

class MTSynthStopReactionOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)
    
class MTSynthTimedOpenReactionValveOp(StationOp):
    def __init__(self, op_model: Union[MTSynthTimedOpenReactionValveOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, duration: float,
                  time_unit: Literal["second", "minute", "hour"]):
        model = MTSynthTimedOpenReactionValveOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.duration = float(duration)
        model.time_unit = time_unit
        model.save()
        return cls(model)
    
    @property
    def duration(self) -> float:
        return self._model_proxy.duration

    @property
    def time_unit(self) -> Literal["second", "minute", "hour"]:
        return self._model_proxy.time_unit

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

''' FILTRATION OPERATIONS '''
class MTSynthFilterOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

class MTSynthDryOp(StationOp):
    def __init__(self, op_model: Union[MTSynthDryOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = MTSynthDryOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
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

class MTSynthDrainOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

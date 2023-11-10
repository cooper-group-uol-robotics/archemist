from typing import Dict, Union, Any, List, Literal, Type
from archemist.core.persistence.models_proxy import ModelProxy, EmbedModelProxy, ListProxy
from .model import (MTSynthesisStationModel,
                    SynthesisCartridgeModel,
                    MTSynthesisPhase,
                    OptiMaxMode,
                    MTSynthLoadCartridgeOpModel,
                    MTSynthDispenseOpModel,
                    MTSynthLiquidDispenseOpModel,
                    MTSynthReactAndWaitOpModel,
                    MTSynthFiltrationDrainOpModel,
                    MTSynthFiltrationVacuumOpModel,
                    MTSyntReactAndSampleOpModel)
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
    def synthesis_phase(self) -> MTSynthesisPhase:
        return self._model_proxy.synthesis_phase

    @synthesis_phase.setter
    def synthesis_phase(self, new_phase: MTSynthesisPhase):
        self._model_proxy.synthesis_phase = new_phase

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
    def horizontal_doors_open(self) -> bool:
        return self._model_proxy.horizontal_doors_open

    @horizontal_doors_open.setter
    def horizontal_doors_open(self, new_state: bool):
        self._model_proxy.horizontal_doors_open = new_state

    @property
    def vertical_doors_open(self) -> bool:
        return self._model_proxy.vertical_doors_open

    @vertical_doors_open.setter
    def vertical_doors_open(self, new_state: bool):
        self._model_proxy.vertical_doors_open = new_state

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
        current_op = self.assigned_op
        if isinstance(current_op, MTSynthReactAndWaitOp):
            if current_op.target_temperature is not None and current_op.target_stirring_speed is not None:
                self.optimax_mode = OptiMaxMode.HEATING_STIRRING
                self.set_reaction_temperature = current_op.target_temperature
                self.set_stirring_speed = current_op.target_stirring_speed
            elif current_op.target_temperature is not None:
                self.optimax_mode = OptiMaxMode.HEATING
                self.set_reaction_temperature = current_op.target_temperature
                self.set_stirring_speed = None
            elif current_op.target_stirring_speed is not None:
                self.optimax_mode = OptiMaxMode.STIRRING
                self.set_reaction_temperature = None
                self.set_stirring_speed = current_op.target_stirring_speed

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        current_op = self.assigned_op
        if isinstance(current_op, MTSynthReactAndWaitOp):
            self.optimax_mode =None
        super().complete_assigned_op(outcome, results)



''' ==== Station Operation Descriptors ==== '''
''' DOOR OPERATIONS '''
class MTSynthHOpenDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

class MTSynthHCloseDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

class MTSynthVOpenDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

class MTSynthVCloseDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

''' SOLID ADDITION OPERATIONS '''
class MTSynthLoadCartridgeOp(StationOp):
    def __init__(self, op_model: Union[MTSynthLoadCartridgeOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, cartridge_index: int):
        model = MTSynthLoadCartridgeOpModel()
        model.cartridge_index = cartridge_index
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)
    
    @property
    def cartridge_index(self) -> int:
        return self._model_proxy.cartridge_index

class MTSynthUnloadCartridgeOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

class MTSynthDispenseSolidOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthDispenseOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  solid_name: str,
                  dispense_mass: float,
                  dispense_unit: Literal["g", "mg", "ug"]):
        model = MTSynthDispenseOpModel()
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
    def __init__(self, station_op_model: Union[MTSynthLiquidDispenseOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)


    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  liquid_name: str, 
                  dispense_volume: float,
                  dispense_unit: Literal["L", "mL", "uL"])
        model = MTSynthLiquidDispenseOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_sample = target_sample.model
        model.liquid_name = liquid_name
        model.dispense_volume = dispense_volume
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
class MTSynthReactAndWaitOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthReactAndWaitOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  target_temperature: int,
                  target_stirring_speed: int,
                  wait_duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = MTSynthReactAndWaitOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_temperature = int(target_temperature) if target_temperature else None
        model.target_stirring_speed = int(target_stirring_speed) if target_stirring_speed else None
        model.wait_duration = int(wait_duration) if wait_duration else None
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

class MTSynthReactAndSampleOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSyntReactAndSampleOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  target_temperature: int,
                  target_stirring_speed: int):
        model = MTSyntReactAndSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.target_temperature = int(target_temperature) if target_temperature else None
        model.target_stirring_speed = int(target_stirring_speed) if target_stirring_speed else None
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

''' FILTERATIO OPERATIONS '''
class MTSynthFiltrationDrainOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthFiltrationDrainOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = MTSynthFiltrationDrainOpModel()
        model.target_sample = target_sample.model
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

class MTSynthFiltrationVacuumOp(StationSampleOp):
    def __init__(self, op_model: Union[MTSynthFiltrationDrainOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  duration: int,
                  time_unit: Literal["second", "minute", "hour"]):
        model = MTSynthFiltrationVacuumOpModel()
        model.target_sample = target_sample.model
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

class MTSynthFiltrationStopOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=MTSynthesisStation.__name__)
        model.save()
        return cls(model)

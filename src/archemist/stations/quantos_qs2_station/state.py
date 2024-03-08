from .model import (QuantosCartridgeModel, QuantosSolidDispenserQS2Model,
                    QuantosDispenseOpModel, QuantosMoveCarouselOpModel)
from archemist.core.persistence.models_proxy import ModelProxy, EmbedModelProxy, ListProxy
from archemist.core.models.station_op_model import StationOpModel
from archemist.core.state.station_op_result import MaterialOpResult
from archemist.core.state.station import Station
from archemist.core.state.sample import Sample
from archemist.core.state.station_op import StationOp, StationSampleOp
from archemist.core.util.enums import OpOutcome
from typing import Dict, Any, Union, List, Literal, Optional


class QuantosCartridge:
    def __init__(self, op_specs_model: Union[QuantosCartridgeModel, EmbedModelProxy]):
        self._model_proxy = op_specs_model

    @classmethod
    def from_dict(cls, cartridge_dict: Dict[str, Any]):
        model = QuantosCartridgeModel()
        model.associated_solid = cartridge_dict["associated_solid"]
        model.hotel_index = cartridge_dict["hotel_index"]
        model.remaining_dosages = cartridge_dict["remaining_dosages"]
        return cls(model)

    @property
    def model(self) -> QuantosCartridgeModel:
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

    def is_consumed(self) -> bool:
        return self._model_proxy.remaining_dosages == 0

    @property
    def blocked(self) -> bool:
        return self._model_proxy.blocked

    @property
    def remaining_dosages(self) -> int:
        return self._model_proxy.remaining_dosages

    @remaining_dosages.setter
    def remaining_dosages(self, new_value: int):
        self._model_proxy.remaining_dosages = new_value


class QuantosSolidDispenserQS2(Station):
    def __init__(self, station_model: Union[QuantosSolidDispenserQS2Model, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = QuantosSolidDispenserQS2Model()
        cls._set_model_common_fields(model, station_dict)
        for cartridge_dict in station_dict['properties']['cartridges']:
            cartridge = QuantosCartridge.from_dict(cartridge_dict)
            model.cartridges.append(cartridge.model)
        model.save()
        return cls(model)

    @property
    def carousel_pos(self) -> int:
        return self._model_proxy.carousel_pos

    @carousel_pos.setter
    def carousel_pos(self, new_pos: int):
        self._model_proxy.carousel_pos = new_pos

    @property
    def cartridges(self) -> List[QuantosCartridge]:
        return ListProxy(self._model_proxy.cartridges, QuantosCartridge)

    @property
    def loaded_cartridge(self) -> QuantosCartridge:
        cartridge_index = self._model_proxy.loaded_cartridge_index
        if cartridge_index is not None:
            return self.cartridges[cartridge_index]

    @property
    def door_open(self) -> bool:
        return self._model_proxy.door_open

    @door_open.setter
    def door_open(self, new_state: bool):
        self._model_proxy.door_open = new_state

    def _dispense(self):
        loaded_cartridge = self.loaded_cartridge
        op = self.assigned_op
        if loaded_cartridge.associated_solid == op.solid_name:
            if not loaded_cartridge.is_consumed():
                if not loaded_cartridge.blocked:
                    solid = self.solids_dict[op.solid_name]
                    solid.decrease_mass(op.dispense_mass, op.dispense_unit)
                    loaded_cartridge.remaining_dosages -= 1
                else:
                    self._log_station(
                        f'Current cartridge {loaded_cartridge} is blocked. Cannot Dispense solid!!!')
            else:
                self._log_station(
                    f'the cartridge {loaded_cartridge} does not have any dosages left. please replace it.')
        else:
            self._log_station(
                "dispense operation target is different from the loaded cartridge")

    def load_cartridge(self, solid_name: str):
        for index, cartridge in enumerate(self.cartridges):
            if cartridge.associated_solid == solid_name:
                self._model_proxy.loaded_cartridge_index = index
                break

    def unload_cartridge(self):
        self._model_proxy.loaded_cartridge_index = None

    def add_station_op(self, station_op: type[StationOp]):
        if isinstance(station_op, QuantosDispenseOp) and not self.loaded_cartridge:
            self._log_station(
                'Quantos station do not have a loaded cartridge!!!')
            self._log_station(f'station_op {station_op} cannot be added')
        else:
            return super().add_station_op(station_op)

    def complete_assigned_op(self, outcome: OpOutcome, results: Optional[List[MaterialOpResult]]):
        current_op = self.assigned_op
        if isinstance(current_op, QuantosOpenDoorOp):
            self.door_open = True
        elif isinstance(current_op, QuantosCloseDoorOp):
            self.door_open = False
        elif isinstance(current_op, QuantosMoveCarouselOp):
            self.carousel_pos = current_op.target_pos
        elif isinstance(current_op, QuantosDispenseOp):
            self._dispense()
        super().complete_assigned_op(outcome, results)


''' ==== Station Operation Descriptors ==== '''


class QuantosOpenDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(
            model, associated_station=QuantosSolidDispenserQS2.__name__)
        model.save()
        return cls(model)


class QuantosCloseDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(
            model, associated_station=QuantosSolidDispenserQS2.__name__)
        model.save()
        return cls(model)


class QuantosMoveCarouselOp(StationOp):
    def __init__(self, op_model: Union[QuantosMoveCarouselOpModel, ModelProxy]):
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_pos: int):
        model = QuantosMoveCarouselOpModel()
        cls._set_model_common_fields(
            model, associated_station=QuantosSolidDispenserQS2.__name__)
        model.target_pos = target_pos
        model.save()
        return cls(model)

    @property
    def target_pos(self) -> int:
        return self._model_proxy.target_pos


class QuantosDispenseOp(StationSampleOp):
    def __init__(self, op_model: Union[QuantosDispenseOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  solid_name: str,
                  dispense_mass: float,
                  dispense_unit: Literal["g", "mg", "ug"]):
        model = QuantosDispenseOpModel()
        cls._set_model_common_fields(
            model, associated_station=QuantosSolidDispenserQS2.__name__)
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

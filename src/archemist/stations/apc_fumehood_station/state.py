from typing import Dict, Union, Any, List, Literal, Type
from archemist.core.persistence.models_proxy import ModelProxy, EmbedModelProxy, ListProxy
from .model import APCCartridgeModel, APCFumehoodStationModel, APCDispenseSolidOpModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOp, StationSampleOp, StationOpModel
from archemist.core.state.sample import Sample
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.util.enums import OpOutcome

class APCCartridge:
    def __init__(self, cartridge_model: Union[APCCartridgeModel, EmbedModelProxy]):
        self._model_proxy = cartridge_model

    @classmethod
    def from_dict(cls, cartridge_dict: Dict[str, Any]):
        model = APCCartridgeModel()
        model.associated_solid = cartridge_dict["associated_solid"]
        model.hotel_index = cartridge_dict["hotel_index"]
        return cls(model)

    @property
    def model(self) -> APCCartridgeModel:
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

class APCFumehoodStation(Station):
    def __init__(self, station_model: Union[APCFumehoodStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = APCFumehoodStationModel()
        cls._set_model_common_fields(model, station_dict)
        for cartridge_dict in station_dict['properties']['cartridges']:
            cartridge = APCCartridge.from_dict(cartridge_dict)
            model.cartridges.append(cartridge.model)
        model.save()
        return cls(model)
    
    @property
    def cartridges(self) -> List[APCCartridge]:
        return ListProxy(self._model_proxy.cartridges, APCCartridge)

    @property
    def loaded_cartridge(self) -> APCCartridge:
        cartridge_index = self._model_proxy.hotel_index
        if cartridge_index is not None:
            return self.cartridges[cartridge_index]

    @property
    def sash_open(self) -> bool:
        return self._model_proxy.sash_open

    @sash_open.setter
    def sash_open(self, new_state: bool):
        self._model_proxy.sash_open = new_state

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        op = self.assigned_op
        if isinstance(op, APCDispenseSolidOp):
            solid = self.solids_dict[op.solid_name]
            solid.decrease_mass(op.dispense_mass, op.dispense_unit)
            self.loaded_cartridge.depleted = True
        elif isinstance(op, APCOpenSashOp):
            self.sash_open = True
        elif isinstance(op, APCCloseSashOp):
            self.sash_open = False
        super().complete_assigned_op(outcome, results)


class APCDispenseSolidOp(StationSampleOp):
    def __init__(self, op_model: Union[APCDispenseSolidOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  solid_name: str,
                  dispense_mass: float,
                  dispense_unit: Literal["g", "mg", "ug"]):
        model = APCDispenseSolidOpModel()
        cls._set_model_common_fields(model, associated_station=APCFumehoodStation.__name__)
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
    
class APCOpenSashOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCFumehoodStation.__name__)
        model.save()
        return cls(model)
    
class APCCloseSashOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCFumehoodStation.__name__)
        model.save()
        return cls(model)
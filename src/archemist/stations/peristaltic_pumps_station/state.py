from .model import PeristalticPumpsStationModel, PPLiquidDispenseOpModel
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station
from archemist.core.state.sample import Sample
from archemist.core.state.station_op import StationSampleOp
from archemist.core.state.station_op_result import StationOpResult
from typing import Dict, Union, Literal, Type, List
from archemist.core.util.enums import OpOutcome


class PeristalticPumpsStation(Station):
    def __init__(self, station_model: Union[PeristalticPumpsStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = PeristalticPumpsStationModel()
        cls._set_model_common_fields(model, station_dict)
        model.liquid_pump_map = station_dict['properties']['liquid_pump_map']
        model.save()
        return cls(model)

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        op = self.assigned_op
        if isinstance(op, PPLiquidDispenseOp):
            liquid = self.liquids_dict[op.liquid_name]
            liquid.decrease_volume(op.dispense_volume, op.dispense_unit)
        super().complete_assigned_op(outcome, results)

    @property
    def liquid_pump_map(self) -> Dict[str, int]:
        return self._model_proxy.liquid_pump_map


''' ==== Station Operation Descriptors ==== '''


class PPLiquidDispenseOp(StationSampleOp):
    def __init__(self, station_op_model: Union[PPLiquidDispenseOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  liquid_name: str,
                  dispense_volume: float,
                  dispense_unit: Literal["L", "mL", "uL"]):
        model = PPLiquidDispenseOpModel()
        cls._set_model_common_fields(
            model, associated_station=PeristalticPumpsStation.__name__)
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

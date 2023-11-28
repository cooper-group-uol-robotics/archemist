from .model import DiaphragmPumpDispenseVolumeOpModel

from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.sample import Sample
from archemist.core.state.station import Station, StationModel
from archemist.core.state.station_op import StationSampleOp
from archemist.core.state.station_op_result import StationOpResult, MaterialOpResult
from typing import Dict, Union, Literal, Type, List
from archemist.core.util.enums import OpOutcome


class DiaphragmPumpStation(Station):
    def __init__(self, station_model: Union[StationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = StationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        if results:
            result = results[0]
            if isinstance(result, MaterialOpResult):
                for index, liquid_name in enumerate(result.material_names):
                    liquid = self.liquids_dict[liquid_name]
                    liquid.decrease_volume(result.amounts[index], result.units[index])
        super().complete_assigned_op(outcome, results)


class DiaphragmPumpDispenseVolumeOp(StationSampleOp):
    def __init__(self, station_op_model: Union[DiaphragmPumpDispenseVolumeOpModel, ModelProxy]) -> None:
        super().__init__(station_op_model)


    @classmethod
    def from_args(cls,
                  target_sample: Sample,
                  liquid_name: str, 
                  dispense_volume: float,
                  dispense_unit: Literal["L", "mL", "uL"]):
        model = DiaphragmPumpDispenseVolumeOpModel()
        cls._set_model_common_fields(model, associated_station=DiaphragmPumpStation.__name__)
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

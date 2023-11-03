from bson.objectid import ObjectId
from .model import SolubilityState, SolubilityOpResultModel
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.models.station_model import StationModel
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.station import Station
from archemist.core.state.sample import Sample
from archemist.core.state.station_op import StationSampleOpDescriptor, StationSampleOpDescriptorModel
from datetime import datetime
from typing import Dict, List, Union

''' ==== Station Description ==== '''
class SolubilityStation(Station):
    def __init__(self, station_model: Union[StationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = StationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)

''' ==== Station Operation Descriptors ==== '''
class CheckSolubilityOp(StationSampleOpDescriptor):
    def __init__(self, op_model: Union[StationSampleOpDescriptorModel,ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpDescriptorModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=SolubilityStation.__name__)
        model.save()
        return cls(model)
    
class SolubilityOpResult(StationOpResult):
    def __init__(self, result_model: Union[SolubilityOpResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls, origin_op: ObjectId,
                  solubility_state: SolubilityState,
                  result_filename: str):
        model = SolubilityOpResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.solubility_state = solubility_state
        model.result_filename = result_filename
        model.save()
        return cls(model)

    @property
    def result_filename(self) -> str:
        return self._model_proxy.result_filename
    
    @property
    def solubility_state(self) -> SolubilityState:
        return self._model_proxy.solubility_state






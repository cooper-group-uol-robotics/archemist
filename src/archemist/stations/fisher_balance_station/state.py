from .model import FisherWeighResultModel
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationSampleOpModel
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationSampleOp
from archemist.core.state.sample import Sample
from typing import Dict, Union, Literal
from bson.objectid import ObjectId

''' ==== Station Description ==== '''


class FisherWeightingStation(Station):
    def __init__(self, station_model: Union[StationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = StationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)


''' ==== Station Operation Descriptors ==== '''


class FisherWeighOp(StationSampleOp):
    def __init__(self, op_model: StationSampleOpModel):
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=FisherWeightingStation.__name__)
        model.save()
        return cls(model)


class FisherWeighResult(StationOpResult):
    def __init__(self, result_model: Union[FisherWeighResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls,
                  origin_op: ObjectId,
                  reading_value: float,
                  unit: Literal["g", "mg", "ug"]):
        model = FisherWeighResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.reading_value = reading_value
        model.unit = unit
        model.save()
        return cls(model)

    @property
    def reading_value(self) -> float:
        return self._model_proxy.reading_value

    @property
    def unit(self) -> Literal["g", "mg", "ug"]:
        return self._model_proxy.unit

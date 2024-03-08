from bson.objectid import ObjectId
from archemist.core.state.sample import Sample
from archemist.core.models.station_op_model import StationSampleOpModel
from archemist.core.state.station_op_result import StationOpResult
from .model import (LightBoxStationModel,
                    LBAnalyseRGBResultModel,
                    LBAnalyseLABResultModel)
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationSampleOp
from typing import List, Union, Dict
import math


''' ==== Station Description ==== '''


class LightBoxStation(Station):
    def __init__(self, station_model: Union[LightBoxStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = LightBoxStationModel()
        cls._set_model_common_fields(model, station_dict)
        if "rgb_target_index" in station_dict["properties"]:
            model.rgb_target_index = int(
                station_dict["properties"]["rgb_target_index"])
        if "lab_target_index" in station_dict["properties"]:
            model.lab_target_index = float(
                station_dict["properties"]["lab_target_index"])
        model.save()
        return cls(model)

    @property
    def rgb_target_index(self) -> int:
        return self._model_proxy.rgb_target_index

    @property
    def lab_target_index(self) -> float:
        return self._model_proxy.lab_target_index


''' ==== Station Operation Descriptors ==== '''


class LBSampleAnalyseRGBOp(StationSampleOp):
    def __init__(self, op_model: Union[StationSampleOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(
            model, associated_station=LightBoxStation.__name__)
        model.save()
        return cls(model)


class LBAnalyseRGBResult(StationOpResult):
    def __init__(self, result_model: Union[LBAnalyseRGBResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls,
                  origin_op: ObjectId,
                  r_value: int,
                  g_value: int,
                  b_value: int,
                  color_index: int,
                  target_index: int,
                  result_filename: str):
        model = LBAnalyseRGBResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.red_intensity = r_value
        model.green_intensity = g_value
        model.blue_intensity = b_value
        model.color_index = color_index
        model.color_diff = abs(target_index - color_index)
        model.result_filename = result_filename
        model.save()
        return cls(model)

    @property
    def result_filename(self) -> str:
        return self._model_proxy.result_filename

    @property
    def red_intensity(self) -> int:
        return self._model_proxy.red_intensity

    @property
    def green_intensity(self) -> int:
        return self._model_proxy.green_intensity

    @property
    def blue_intensity(self) -> int:
        return self._model_proxy.blue_intensity

    @property
    def color_index(self) -> int:
        return self._model_proxy.color_index

    @property
    def color_diff(self) -> int:
        return self._model_proxy.color_diff


class LBSampleAnalyseLABOp(StationSampleOp):
    def __init__(self, op_model: Union[StationSampleOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(
            model, associated_station=LightBoxStation.__name__)
        model.save()
        return cls(model)


class LBAnalyseLABResult(StationOpResult):
    def __init__(self, result_model: Union[LBAnalyseLABResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls,
                  origin_op: ObjectId,
                  l_value: float,
                  a_value: float,
                  b_value: float,
                  color_index: float,
                  target_index: float,
                  result_filename: str):
        model = LBAnalyseLABResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.l_star_value = l_value
        model.a_star_value = a_value
        model.b_star_value = b_value
        model.color_index = color_index
        model.color_diff = math.sqrt(color_index**2 - target_index**2)
        model.result_filename = result_filename
        model.save()
        return cls(model)

    @property
    def result_filename(self) -> str:
        return self._model_proxy.result_filename

    @property
    def l_star_value(self) -> float:
        return self._model_proxy.l_star_value

    @property
    def a_star_value(self) -> float:
        return self._model_proxy.a_star_value

    @property
    def b_star_value(self) -> float:
        return self._model_proxy.b_star_value

    @property
    def color_index(self) -> float:
        return self._model_proxy.color_index

    @property
    def color_diff(self) -> float:
        return self._model_proxy.color_diff

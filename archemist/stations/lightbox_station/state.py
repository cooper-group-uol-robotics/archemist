from .model import SampleColorOpDescriptorModel
from archemist.core.models.station_model import StationModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid, Solid
from typing import Dict, List
from datetime import datetime


""" ==== Station Description ==== """


class LightBoxStation(Station):
    def __init__(self, station_model: StationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = StationModel()
        cls._set_model_common_fields(station_dict, model)
        model._module = cls.__module__
        model.save()
        return cls(model)


""" ==== Station Operation Descriptors ==== """


class SampleColorOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: SampleColorOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = SampleColorOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def result_filename(self) -> str:
        return self._model.result_filename

    @property
    def red_intensity(self) -> int:
        return self._model.red_intensity

    @property
    def green_intensity(self) -> int:
        return self._model.green_intensity

    @property
    def blue_intensity(self) -> int:
        return self._model.blue_intensity

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if "result_filename" in kwargs:
            self._model.result_filename = kwargs["result_filename"]
        else:
            print("missing result_file!!")
        if all(
            karg in kwargs
            for karg in ["red_intensity", "green_intensity", "blue_intensity"]
        ):
            self._model.red_intensity = kwargs["red_intensity"]
            self._model.green_intensity = kwargs["green_intensity"]
            self._model.blue_intensity = kwargs["blue_intensity"]
        else:
            print("missing one or all color intensity values")

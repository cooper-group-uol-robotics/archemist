from .model import LightBoxStationModel, SampleColorOpRGBDescriptorModel, SampleColorLABOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Liquid,Solid
from typing import Dict, List
from datetime import datetime
import math


''' ==== Station Description ==== '''
class LightBoxStation(Station):
    def __init__(self, station_model: LightBoxStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = LightBoxStationModel()
        cls._set_model_common_fields(station_dict,model)
        if "rgb_target" in station_dict["parameters"]:
            model.rgb_target = int(station_dict["parameters"]["rgb_target"])
        if "lab_target" in station_dict["parameters"]:
            model.lab_target = float(station_dict["parameters"]["lab_target"])
        model._module = cls.__module__
        model.save()
        return cls(model)
    
    @property
    def rgb_target(self) -> int:
        return self._model.rgb_target
    
    @property
    def lab_target(self) -> float:
        return self._model.lab_target
    
    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if kwargs.get("color_index") is not None:
            if isinstance(current_op, SampleColorRGBOpDescriptor):
                diff = -1*abs(kwargs["color_index"] - self.rgb_target)
            elif isinstance(current_op, SampleColorLABOpDescriptor):
                diff = -1*math.sqrt(kwargs["color_index"]**2 - self.lab_target**2)
            super().complete_assigned_station_op(success, target_diff=diff, **kwargs)
        else:
            super().complete_assigned_station_op(success, **kwargs)

''' ==== Station Operation Descriptors ==== '''
class SampleColorRGBOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: SampleColorOpRGBDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = SampleColorOpRGBDescriptorModel()
        cls._set_model_common_fields(model, associated_station=LightBoxStation.__name__, **kwargs)
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
    
    @property
    def color_index(self) -> int:
        return self._model.color_index
    
    @property
    def target_diff(self) -> int:
        return self._model.target_diff

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'result_filename' in kwargs:
            self._model.result_filename = kwargs['result_filename']
        else:
            print('missing result_file!!')
        if all(karg in kwargs for karg in ['red_intensity','green_intensity','blue_intensity', 'color_index', 'target_diff']):
            self._model.red_intensity = kwargs['red_intensity']
            self._model.green_intensity = kwargs['green_intensity']
            self._model.blue_intensity = kwargs['blue_intensity']
            self._model.color_index = kwargs['color_index']
            self._model.target_diff = kwargs['target_diff']
        else:
            print('missing one or all color intensity values. adding defaults')
            self._model.red_intensity = 0
            self._model.green_intensity = 0
            self._model.blue_intensity = 0
            self._model.color_index = 0
            self._model.target_diff = 0

class SampleColorLABOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: SampleColorLABOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = SampleColorLABOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=LightBoxStation.__name__, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

    @property
    def result_filename(self) -> str:
        return self._model.result_filename

    @property
    def l_value(self) -> float:
        return self._model.l_value

    @property
    def a_value(self) -> float:
        return self._model.a_value

    @property
    def b_value(self) -> float:
        return self._model.b_value
    
    @property
    def color_index(self) -> float:
        return self._model.color_index
    
    @property
    def target_diff(self) -> float:
        return self._model.target_diff

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'result_filename' in kwargs:
            self._model.result_filename = kwargs['result_filename']
        else:
            print('missing result_file!!')
        if all(karg in kwargs for karg in ['l_value','a_value','b_value', 'color_index', 'target_diff']):
            self._model.l_value = kwargs['l_value']
            self._model.a_value = kwargs['a_value']
            self._model.b_value = kwargs['b_value']
            self._model.color_index = kwargs['color_index']
            self._model.target_diff = kwargs['target_diff']
        else:
            print('missing one or all color intensity values. adding defaults')
            self._model.l_value = 0
            self._model.a_value = 0
            self._model.b_value = 0
            self._model.color_index = 0
            self._model.target_diff = 0
    
    

    

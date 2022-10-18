from archemist.state.station import StationModel,Station,StationOpDescriptorModel,StationOpDescriptor
from archemist.state.material import Liquid,Solid
from typing import Dict, List
from mongoengine import fields
from datetime import datetime


''' ==== Station Description ==== '''
class LightBoxStation(Station):
    def __init__(self, station_model: StationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = StationModel()
        cls._set_model_common_fields(station_dict,model)
        model._module = cls.__module__
        model.save()
        return cls(model)

''' ==== Station Operation Descriptors ==== '''
class SampleColorOpDescriptorModel(StationOpDescriptorModel):
    result_filename = fields.StringField()
    red_intensity = fields.IntField(min_value=0, max_value=255)
    green_intensity = fields.IntField(min_value=0, max_value=255)
    blue_intensity = fields.IntField(min_value=0, max_value=255)

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
        if 'result_filename' in kwargs:
            self._model.result_filename = kwargs['result_filename']
        else:
            print('missing result_file!!')
        self._model.red_intensity = kwargs['red_intensity']
        self._model.green_intensity = kwargs['green_intensity']
        self._model.blue_intensity = kwargs['blue_intensity']

    
    

    

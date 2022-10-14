from bson.objectid import ObjectId
from archemist.state.station import StationModel, Station, StationOpDescriptorModel, StationOpDescriptor
from archemist.state.material import Liquid, Solid
from mongoengine import fields
from typing import List, Any
from archemist.exceptions.exception import InvalidLiquidError
from datetime import datetime

class PeristalticPumpStationModel(StationModel):
    pump_liquid_map = fields.DictField(required=True)

class PeristalticLiquidDispensing(Station):
    def __init__(self, station_model: PeristalticPumpStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_document: dict, liquids: List[Liquid], solids: List[Solid]):
        model = PeristalticPumpStationModel()
        cls._set_model_common_fields(station_document, model)
        model._type = cls.__name__
        parameters = station_document['parameters']
        for _, pump_id in parameters['liquid_pump_map'].items():
                for liquid in liquids:
                    if liquid.pump_id == pump_id:
                        model.pump_liquid_map[pump_id] = liquid.model.id
        model.save()
        return cls(model)


    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = PeristalticPumpStationModel.objects.get(id=object_id)
        return cls(model)

    def get_liquid(self, pump_id: str):
        liquid_obj_id = self._model.pump_liquid_map[pump_id]
        return Liquid.from_object_id(liquid_obj_id)

    def get_pump_id(self, liquid_name: str):
        # assuming only single liquid per pump
        pump_liquid_map = self._model.pump_liquid_map
        for pump_id, liquid_obj_id in pump_liquid_map.items():
            if liquid_name == Liquid.from_object_id(liquid_obj_id).name:
                return pump_id
            else:
                raise InvalidLiquidError(self.__class__.__name__)

    def add_to_pump_liquid_level(self, pump_id: str, added_value: float):
        modified_liquid = self.get_liquid(pump_id)
        modified_liquid.volume = modified_liquid.volume + added_value/1000 #assume value in ml

    def add_liquid(self, liquid_nam: str, added_volume: float):
        pump_id = self.get_pump_id(liquid_nam)
        self.add_to_pump_liquid_level(pump_id, added_volume)

    def reduce_pump_liquid_level(self, pump_id: str, value: float):
        modified_liquid = self.get_liquid(pump_id)
        modified_liquid.volume = modified_liquid.volume - value/1000 # assume value in ml

    def dispense_liquid(self, liquid_nam: str, dispensed_volume: float):
        pump_id = self.get_pump_id(liquid_nam)
        self.reduce_pump_liquid_level(pump_id, dispensed_volume)

    def finish_station_op(self, success: bool, **kwargs):
        current_op = self.get_station_op()
        if isinstance(current_op, PeristalticPumpOpDescriptor):
            if success:
                if 'actual_dispensed_volume' in kwargs:
                    self.dispense_liquid(current_op.liquid_name, kwargs['actual_dispensed_volume'])
                else:
                    self.dispense(current_op.liquid_name, current_op.dispense_volume)
        super().finish_station_op(success, **kwargs)

''' ==== Station Operation Descriptors ==== '''

class PeristalticPumpOpDescriptorModel(StationOpDescriptorModel):
    liquid_name = fields.StringField(required=True)
    dispense_volume = fields.FloatField(min_value=0, required=True)
    actual_dispensed_volume = fields.FloatField(min_value=0)

class PeristalticPumpOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: PeristalticPumpOpDescriptorModel):
        self._model = op_model


    @classmethod
    def from_args(cls, liquid_name: str, dispense_volume: float):
        model = PeristalticPumpOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        model.liquid_name = liquid_name
        model.dispense_volume = dispense_volume
        return cls(model)
        

    @property
    def liquid_name(self) -> str:
        return self._model.liquid_name

    @property
    def dispense_volume(self) -> float:
        return self._model.dispense_volume

    @property
    def actual_dispensed_volume(self) -> float:
        if self._model.has_result and self._model.was_successful:
            return self._model.actual_dispensed_volume

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'actual_dispensed_volume' in kwargs:
            self._model.actual_dispensed_volume = kwargs['actual_dispensed_volume']
        else:
            print('missing actual dispensed volume!!')

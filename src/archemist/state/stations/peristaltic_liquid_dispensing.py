from bson.objectid import ObjectId
from archemist.state.station import StationModel, Station, StationOpDescriptorModel, StationOpDescriptor
from archemist.state.material import Liquid, Solid
from mongoengine import fields
from typing import List, Any
from archemist.exceptions.exception import InvalidLiquidError

class PeristalticPumpStationModel(StationModel):
    pump_liquid_map = fields.DictField(required=True)

class PeristalticLiquidDispensing(Station):
    def __init__(self, station_model: PeristalticPumpStationModel) -> None:
        self._model = station_model

    @classmethod
    def from_dict(cls, station_document: dict, liquids: List[Liquid], solids: List[Solid]):
        model = PeristalticPumpStationModel()
        cls._set_model_common_fields(station_document, model)
        parameters = station_document['parameters']
        for _, pumpId in parameters['liquid_pump_map'].items():
                for liquid in liquids:
                    if liquid.pump_id == pumpId:
                        model.pump_liquid_map[pumpId] = liquid.model.id
        model.save()
        return cls(model)


    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = PeristalticPumpStationModel.objects.get(id=object_id)
        return cls(model)

    def get_liquid(self, pumpId: str):
        liquid_obj_id = self._model.pump_liquid_map[pumpId]
        return Liquid.from_object_id(liquid_obj_id)

    def get_pump_id(self, liquid_name: str):
        # assuming only single liquid per pump
        pump_liquid_map = self._model.pump_liquid_map
        for pumpId, liquid_obj_id in pump_liquid_map.items():
            if liquid_name == Liquid.from_object_id(liquid_obj_id).name:
                return pumpId
            else:
                raise InvalidLiquidError(self.__class__.__name__)

    def add_to_pump_liquid_level(self, pumpId: str, added_value: float):
        modified_liquid = self.get_liquid(pumpId)
        modified_liquid.volume = modified_liquid.volume + added_value/1000 #assume value in ml

    def add_liquid(self, liquid_nam: str, added_volume: float):
        pumpId = self.get_pump_id(liquid_nam)
        self.add_to_pump_liquid_level(pumpId, added_volume)

    def reduce_pump_liquid_level(self, pumpId: str, value: float):
        modified_liquid = self.get_liquid(pumpId)
        modified_liquid.volume = modified_liquid.volume - value/1000 # assume value in ml

    def dispense_liquid(self, liquid_nam: str, dispensed_volume: float):
        pumpId = self.get_pump_id(liquid_nam)
        self.reduce_pump_liquid_level(pumpId, dispensed_volume)

''' ==== Station Operation Descriptors ==== '''

class PeristalticPumpOpDescriptorModel(StationOpDescriptorModel):
    liquid_name = fields.StringField(required=True)
    dispense_volume = fields.FloatField(min_value=0, required=True)
    actual_dispensed_volume = fields.FloatField(min_value=0)

class PeristalticPumpOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: PeristalticPumpOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_model(cls, op_model: PeristalticPumpOpDescriptorModel):
        return cls(op_model)

    @classmethod
    def from_args(cls, **kwargs):
        model = PeristalticPumpOpDescriptorModel()
        model.type = cls.__name__
        model.module = cls.__module__
        model.liquid_name = kwargs['liquid']
        model.dispense_volume = kwargs['volume']
        return cls(model)
        

    @property
    def liquid_name(self):
        return self._model.liquid_name

    @property
    def dispense_volume(self):
        return self._model.dispense_volume

    @property
    def actual_dispensed_volume(self):
        return self._model.actual_dispensed_volume

    @actual_dispensed_volume.setter
    def actual_dispensed_volume(self, dispensed_volume: float):
        self._model.actual_dispensed_volume = dispensed_volume

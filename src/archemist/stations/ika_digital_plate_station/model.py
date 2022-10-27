from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class IKAMode(Enum):
    HEATING = 1
    STIRRING = 2
    HEATINGSTIRRING = 3

class IkaPlateDigitalModel(StationModel):
    mode = fields.EnumField(IKAMode, null=True)
    current_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    target_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    current_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)
    external_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    viscosity_trend = fields.FloatField(null=True)
    target_duration = fields.FloatField(null=True)

class IKAOpDescriptorModel(StationOpDescriptorModel):
    target_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)
    target_duration = fields.FloatField(null=True)
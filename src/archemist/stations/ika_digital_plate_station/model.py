from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationBatchOpModel
from mongoengine import fields
from enum import Enum


class IKADigitalPlateMode(Enum):
    HEATING = 1
    STIRRING = 2
    HEATING_STIRRING = 3


class IkaDigitalPlateStationModel(StationModel):
    mode = fields.EnumField(IKADigitalPlateMode, null=True)
    current_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    current_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)
    external_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    viscosity_trend = fields.FloatField(null=True)


class IKADigitalPlateOpModel(StationBatchOpModel):
    target_temperature = fields.IntField(min_value=0, max_value=500, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)
    duration = fields.IntField(required=True)
    time_unit = fields.StringField(choices=["second", "minute", "hour"], default="second")

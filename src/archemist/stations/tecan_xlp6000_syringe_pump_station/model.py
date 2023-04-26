from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class SyringePumpMode(Enum):
    DISPENSE = 1
    WITHDRAW = 2

class SyringePumpStationModel(StationModel):
    mode = fields.EnumField(SyringePumpMode, null = True)

class SyringePumpOpDescriptorModel(StationOpDescriptorModel):
    port = fields.StringField(required=True)
    volume = fields.FloatField(min_value=0, required=True)
    speed = fields.FloatField(min_value=0)


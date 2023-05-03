from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class SyringePumpStatus(Enum):
    DISPENSE = 1
    WITHDRAW = 2
    JOB_COMPLETE = 3

class SyringePumpStationModel(StationModel):
    machine_status = fields.EnumField(SyringePumpStatus, null = True)

class SyringePumpOpDescriptorModel(StationOpDescriptorModel):
    port = fields.IntField(required=True)
    volume = fields.FloatField(min_value=0, required=True)
    speed = fields.FloatField(min_value=0)


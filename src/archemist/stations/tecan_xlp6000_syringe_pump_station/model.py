from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class SyringePumpStatus(Enum):
    DISPENSE = 1
    JOB_COMPLETE = 2

class SyringePumpStationModel(StationModel):
    machine_status = fields.EnumField(SyringePumpStatus, null = True)

class SyringePumpOpDescriptorModel(StationOpDescriptorModel):
    withdraw_port = fields.IntField(min_value=0, max_value=12,null=True)
    dispense_port = fields.IntField(min_value=0, max_value=12,null=True)
    volume = fields.IntField(min_value=0, null=True)
    speed = fields.IntField(min_value=0, null=True)


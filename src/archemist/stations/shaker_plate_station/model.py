from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum, auto

class ShakerStatus(Enum):
    SHAKING = auto()
    NOT_SHAKING = auto()

class ShakerPlateStationModel(StationModel):
    machine_status = fields.EnumField(ShakerStatus, default=ShakerStatus.NOT_SHAKING)

class ShakeOpDescriptorModel(StationOpDescriptorModel):
    duration = fields.IntField(min_value=0)

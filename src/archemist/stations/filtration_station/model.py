from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class FiltrationStatus(Enum):
    BASEVALVE_OPEN = 0
    VACUUMING_OPEN = 1
    DRAINING_OPEN = 2
    STOP = 3

class FiltrationStationModel(StationModel):
    machine_status = fields.EnumField(FiltrationStatus, null=True)

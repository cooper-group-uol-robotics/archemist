from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class BalanceDoorStatus(Enum):
    DOORS_OPEN = 0
    DOORS_CLOSED = 1

class WeighingStationModel(StationModel):
    machine_status = fields.EnumField(BalanceDoorStatus, null=True)

class WeightOpDescriptorModel(StationOpDescriptorModel):
    mass_reading = fields.FloatField(min_value=0)


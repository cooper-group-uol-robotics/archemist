from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class TurbidityState(Enum):
    DISSOLVED = 0
    PARTIALLY_DISSOLVED = 1
    UNDISSOLVED = 2

class SolubilityOpDescriptorModel(StationOpDescriptorModel):
    turbidity_state = fields.EnumField(TurbidityState)
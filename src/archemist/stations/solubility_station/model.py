from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class SolubilityState(Enum):
    DISSOLVED = 0
    # PARTIALLY_DISSOLVED = 1
    UNDISSOLVED = 1

class SolubilityOpDescriptorModel(StationOpDescriptorModel):
    result_filename = fields.StringField()
    solubility_state = fields.EnumField(SolubilityState, null=True)
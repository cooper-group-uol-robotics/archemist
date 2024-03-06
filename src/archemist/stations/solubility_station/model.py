from archemist.core.models.station_op_result_model import StationOpResultModel
from mongoengine import fields
from enum import Enum


class SolubilityState(Enum):
    DISSOLVED = 0
    UNDISSOLVED = 1


class SolubilityOpResultModel(StationOpResultModel):
    result_filename = fields.StringField()
    solubility_state = fields.EnumField(SolubilityState, required=True)

from archemist.core.models.op_result_model import OpResultModel
from mongoengine import fields
from enum import Enum

class SolubilityState(Enum):
    DISSOLVED = 0
    UNDISSOLVED = 1

class SolubilityOpResultModel(OpResultModel):
    result_filename = fields.StringField()
    solubility_state = fields.EnumField(SolubilityState, required=True)
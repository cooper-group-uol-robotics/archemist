from archemist.core.models.op_result_model import OpResultModel
from mongoengine import fields

class FisherWeighResultModel(OpResultModel):
    reading_value = fields.FloatField(min_value=0)
    unit = fields.StringField(choices=["g", "mg", "ug"], default="g")
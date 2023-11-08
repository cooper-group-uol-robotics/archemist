from archemist.core.models.station_op_result_model import StationOpResultModel
from mongoengine import fields

class FisherWeighResultModel(StationOpResultModel):
    reading_value = fields.FloatField(min_value=0)
    unit = fields.StringField(choices=["g", "mg", "ug"], default="g")
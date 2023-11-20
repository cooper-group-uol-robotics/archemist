from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_result_model import StationOpResultModel
from mongoengine import fields

class ApcWeighingStationModel(StationModel):
    balance_doors_open = fields.BooleanField(default=False)
    vertical_doors_open = fields.BooleanField(default=False)

class ApcWeighResultModel(StationOpResultModel):
    reading_value = fields.FloatField(min_value=0)
    unit = fields.StringField(choices=["g"], default="g")


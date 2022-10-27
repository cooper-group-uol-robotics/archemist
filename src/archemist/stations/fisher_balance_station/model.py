from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields

class FisherWeightOpDescriptorModel(StationOpDescriptorModel):
    weight = fields.FloatField(min_value=0)
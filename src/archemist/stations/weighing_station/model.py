from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields


class WeightOpDescriptorModel(StationOpDescriptorModel):
    weight = fields.FloatField()
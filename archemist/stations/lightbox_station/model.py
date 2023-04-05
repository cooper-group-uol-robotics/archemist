from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields


class SampleColorOpDescriptorModel(StationOpDescriptorModel):
    result_filename = fields.StringField()
    red_intensity = fields.IntField(min_value=0, max_value=255)
    green_intensity = fields.IntField(min_value=0, max_value=255)
    blue_intensity = fields.IntField(min_value=0, max_value=255)

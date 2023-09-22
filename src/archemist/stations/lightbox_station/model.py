from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields


class LightBoxStationModel(StationModel):
    rgb_target = fields.IntField(default=0)
    lab_target = fields.FloatField(default=0)

class SampleColorOpRGBDescriptorModel(StationOpDescriptorModel):
    result_filename = fields.StringField()
    red_intensity = fields.IntField(min_value=0, max_value=255)
    green_intensity = fields.IntField(min_value=0, max_value=255)
    blue_intensity = fields.IntField(min_value=0, max_value=255)
    color_index = fields.IntField()
    target_diff = fields.IntField()

class SampleColorLABOpDescriptorModel(StationOpDescriptorModel):
    result_filename = fields.StringField()
    l_value = fields.FloatField()
    a_value = fields.FloatField()
    b_value = fields.FloatField()
    color_index = fields.FloatField()
    target_diff = fields.FloatField()
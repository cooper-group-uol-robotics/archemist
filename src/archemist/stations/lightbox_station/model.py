from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_result_model import StationOpResultModel
from mongoengine import fields


class LightBoxStationModel(StationModel):
    rgb_target_index = fields.IntField(default=0)
    lab_target_index = fields.FloatField(default=0)  

class LBAnalyseRGBResultModel(StationOpResultModel):
    result_filename = fields.StringField()
    red_intensity = fields.IntField(min_value=0, max_value=255)
    green_intensity = fields.IntField(min_value=0, max_value=255)
    blue_intensity = fields.IntField(min_value=0, max_value=255)
    color_index = fields.IntField()
    color_diff = fields.IntField()    

class LBAnalyseLABResultModel(StationOpResultModel):
    result_filename = fields.StringField()
    l_star_value = fields.FloatField()
    a_star_value = fields.FloatField()
    b_star_value = fields.FloatField()
    color_index = fields.FloatField()
    color_diff = fields.FloatField()
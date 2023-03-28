from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class OptimaxStatus(Enum):
    TEMP_CONTROL = 0
    STIRRING = 1
    T_AND_S = 2



class OptimaxStationModel(StationModel):
    machine_status = fields.EnumField(OptimaxStatus, null=True)
    current_temperature = fields.IntField(min_value=-10, max_value=500, null=True)
    current_stirring_speed = fields.IntField(min_value=0, max_value=1500, null=True)

class OptimaxOpDescriptorModel(StationOpDescriptorModel):
    target_temperature = fields.FloatField(min_value=-10, max_value=500, null=True)
    target_stirring_speed = fields.FloatField(min_value=0, max_value=1500, null=True)
    target_duration = fields.FloatField(null=True)
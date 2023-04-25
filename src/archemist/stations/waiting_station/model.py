from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
    

class WaitingStationModel(StationModel):
    current_occupancy = fields.IntField(min_value=0)

class WaitingStationOpDescriptorModel(StationOpDescriptorModel):
    duration = fields.IntField(min_value=0)
    
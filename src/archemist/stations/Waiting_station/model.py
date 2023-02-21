from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum


class WaitingStationModel(StationModel):
    Number_of_racks_occupied = fields.IntField(min_value=0, max_value=9)

class WaitingStationOpDescriptorModel(StationOpDescriptorModel):

    Batch_start_times = fields.ComplexDateTimeField()
    Batch_target_time = fields.ComplexDateTimeField()
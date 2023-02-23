from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum, auto


class WaitingStationStatus(Enum):
    Not_available=auto()
    Available = auto()
    #number_of_available_slots = ?????? 


class WaitingStationModel(StationModel):
    machine_status = fields.IntField(WaitingStationStatus)
    

class WaitingStationOpDescriptorModel(StationOpDescriptorModel):
    duration = fields.IntField(min_value=0)
    
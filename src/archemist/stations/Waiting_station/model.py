from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum, auto


class WaitingStationStatus(Enum):
    Batch_waiting = auto()
    Not_waiting = auto()
  

class WaitingStationAvailabity(Enum):
    Available = auto()
    Not_available = auto()
    #number_of_available_slots = ?????? 

class WaitingStationModel(StationModel):
    station_status = fields.EnumField(WaitingStationStatus)
    station_availability = fields.EnumField(WaitingStationAvailabity)
    current_occupancy = fields.IntField(min_value=0)

class WaitingStationOpDescriptorModel(StationOpDescriptorModel):
    duration = fields.IntField(min_value=0)
    
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum


class FiltrationValveStatus(Enum):
    VALVE_OPEN = 0
    VALVE_CLOSED = 1


class FiltrationStationModel(StationModel):
    machine_status = fields.EnumField(FiltrationValveStatus, null=True)

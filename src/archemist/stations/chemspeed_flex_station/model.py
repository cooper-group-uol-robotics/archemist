from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class ChemSpeedStatus(Enum):
    DOORS_OPEN = 0
    DOORS_CLOSED = 1
    RUNNING_JOB = 2
    JOB_COMPLETE = 3

class ChemSpeedFlexStationModel(StationModel):
    machine_status = fields.EnumField(ChemSpeedStatus, null=True)
    liquids_dict = fields.DictField(default={})

class CSCSVJobOpDescriptorModel(StationOpDescriptorModel):
    dispense_info = fields.DictField(required=True)
    result_file = fields.StringField()
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields
from enum import Enum

class PXRDStatus(Enum):
    DOORS_OPEN = 0
    DOORS_CLOSED = 1
    RUNNING_JOB = 2
    JOB_COMPLETE = 3
    

class PXRDStationModel(StationModel):
    machine_status = fields.EnumField(PXRDStatus, null=True)

class PXRDAnalysisOpDescriptorModel(StationOpDescriptorModel):
    result_file = fields.StringField()


from archemist.core.models.station_model import StationModel
from archemist.core.models.op_result_model import OpResultModel
from mongoengine import fields
from enum import Enum, auto

class PXRDJobStatus(Enum):
    INVALID = auto()
    RUNNING_JOB = auto()
    JOB_COMPLETE = auto()
    

class PXRDStationModel(StationModel):
    job_status = fields.EnumField(PXRDJobStatus, default=PXRDJobStatus.INVALID)
    door_closed = fields.BooleanField(default=True)

class PXRDAnalysisResultModel(OpResultModel):
    result_filename = fields.StringField()


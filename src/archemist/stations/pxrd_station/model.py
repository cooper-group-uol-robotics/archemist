from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_result_model import StationOpResultModel
from mongoengine import fields
from enum import Enum, auto
from archemist.core.util.location import LocationModel


class PXRDJobStatus(Enum):
    INVALID = auto()
    RUNNING_JOB = auto()
    JOB_COMPLETE = auto()


class PXRDStationModel(StationModel):
    job_status = fields.EnumField(PXRDJobStatus, default=PXRDJobStatus.INVALID)
    door_closed = fields.BooleanField(default=True)
    doors_location = fields.EmbeddedDocumentField(LocationModel, required=True)


class PXRDAnalysisResultModel(StationOpResultModel):
    result_filename = fields.StringField()

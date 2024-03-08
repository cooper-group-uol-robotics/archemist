from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_result_model import StationOpResultModel

from mongoengine import fields
from enum import Enum, auto


class LCMSAnalysisStatus(Enum):
    INVALID = auto()
    RUNNING_ANALYSIS = auto()  # station running analysis
    ANALYSIS_COMPLETE = auto()  # analysis complete


class WatersLCMSStationModel(StationModel):
    batch_inserted = fields.BooleanField(default=False)
    analysis_status = fields.EnumField(
        LCMSAnalysisStatus, default=LCMSAnalysisStatus.INVALID)


class LCMSAnalysisResultModel(StationOpResultModel):
    concentration = fields.FloatField(required=True)
    result_filename = fields.StringField(required=True)

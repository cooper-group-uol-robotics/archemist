from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_result_model import StationOpResultModel

from mongoengine import fields
from enum import Enum, auto


class LCMSAnalysisStatus(Enum):
    INVALID = auto()
    RUNNING_ANALYSIS = auto() # station running analysis
    ANALYSIS_COMPLETE = auto() # analysis complete
class WatersLCMSStationModel(StationModel):
    batch_inserted = fields.BooleanField(default=False)
    sample_index = fields.IntField(min_value=1, default=1)
    analysis_status = fields.EnumField(LCMSAnalysisStatus, default=LCMSAnalysisStatus.INVALID)

class LCMSAnalysisResultModel(StationOpResultModel):
    chemicals = fields.ListField(required=True)
    concentrations = fields.ListField(required=True)
    y_values = fields.ListField(required = True)

    # result_filename = fields.StringField(required=True)
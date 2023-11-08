from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_result_model import StationOpResultModel
from archemist.core.models.station_op_model import (StationBatchOpModel,
                                                    StationOpModel)
from mongoengine import fields
from enum import Enum, auto

class LCMSAutoLoaderStatus(Enum):
    BAY_FREE = auto() # loading bay ready for rack to be added
    BAY_OCCUPIED = auto() # loading bay is occupied 
    BAY_UNAVAILABLE = auto() # loading bay is closed since batch is being analysed

class LCMSAnalysisStatus(Enum):
    INVALID = auto()
    RUNNING_ANALYSIS = auto() # station running analysis
    ANALYSIS_COMPLETE = auto() # analysis complete

class WatersLCMSStationModel(StationModel):
    auto_loader_status = fields.EnumField(LCMSAutoLoaderStatus, default=LCMSAutoLoaderStatus.BAY_FREE)
    analysis_status = fields.EnumField(LCMSAnalysisStatus, default=LCMSAnalysisStatus.INVALID)

class LCMSBayOccupiedOpModel(StationOpModel):
    bay_index = fields.IntField(default=1, min_value=1, max_value=2)

class LCMSBayFreedOpModel(StationOpModel):
    bay_index = fields.IntField(default=1, min_value=1, max_value=2)

class LCMSInsertBatchOpModel(StationBatchOpModel):
    bay_index = fields.IntField(default=1, min_value=1, max_value=2)

class LCMSEjectBatchOpModel(StationBatchOpModel):
    bay_index = fields.IntField(default=1, min_value=1, max_value=2)

class LCMSAnalysisResultModel(StationOpResultModel):
    result_filename = fields.StringField(required=True)
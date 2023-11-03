from archemist.core.models.station_model import StationModel
from archemist.core.models.op_result_model import OpResultModel
from archemist.core.models.station_op_model import (StationBatchOpDescriptorModel,
                                                    StationOpDescriptorModel)
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

class LCMSBayOccupiedOpModel(StationOpDescriptorModel):
    bay_index = fields.IntField(default=1, min_value=1, max_value=2)

class LCMSBayFreedOpModel(StationOpDescriptorModel):
    bay_index = fields.IntField(default=1, min_value=1, max_value=2)

class LCMSInsertBatchOpModel(StationBatchOpDescriptorModel):
    bay_index = fields.IntField(default=1, min_value=1, max_value=2)

class LCMSEjectBatchOpModel(StationBatchOpDescriptorModel):
    bay_index = fields.IntField(default=1, min_value=1, max_value=2)

class LCMSAnalysisResultModel(OpResultModel):
    result_filename = fields.StringField(required=True)
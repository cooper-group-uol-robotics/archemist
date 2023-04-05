# External
from mongoengine import fields
from enum import Enum, auto

# Core
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel


class LCMSStatus(Enum):
    BATCH_LOADED = auto()  # rack loaded inside the LCMS
    BATCH_READY_FOR_COLLECTION = (
        auto()
    )  # rack outside ready to be collected by the robot
    RUNNING_ANALYSIS = auto()  # station running analysis
    ANALYSIS_COMPLETE = auto()  # analysis complete


class WatersLCMSStationModel(StationModel):
    machine_status = fields.EnumField(LCMSStatus, null=True)


class LCMSOpModel(StationOpDescriptorModel):
    rack = fields.IntField(default=1, min_value=1, max_value=2)

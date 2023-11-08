from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationLotOpModel
from mongoengine import fields
from enum import Enum, auto

class ChemSpeedJobStatus(Enum):
    INVALID = auto()
    RUNNING_JOB = auto()
    JOB_COMPLETE = auto()

class ChemSpeedFlexStationModel(StationModel):
    job_status = fields.EnumField(ChemSpeedJobStatus, default=ChemSpeedJobStatus.INVALID)
    door_closed = fields.BooleanField(default=True)

class CSLiquidDispenseOpModel(StationLotOpModel):
    dispense_table = fields.DictField(required=True)
    dispense_unit = fields.StringField(choices=["L", "mL", "uL"], default="mL")
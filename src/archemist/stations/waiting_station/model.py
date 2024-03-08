from archemist.core.models.station_op_model import StationLotOpModel
from mongoengine import fields


class WaitOpModel(StationLotOpModel):
    duration = fields.IntField(min_value=0)
    time_unit = fields.StringField(
        choices=["second", "minute", "hour"], default="second")

from mongoengine import fields
from archemist.core.models.station_op_model import StationSampleOpModel


class APCDryProductOpModel(StationSampleOpModel):
    duration = fields.IntField(required=True)
    time_unit = fields.StringField(
        choices=["second", "minute", "hour"], default="minute")

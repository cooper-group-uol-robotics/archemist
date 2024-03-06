from archemist.core.models.station_op_model import StationSampleOpModel
from mongoengine import fields


class DiaphragmPumpDispenseVolumeOpModel(StationSampleOpModel):
    liquid_name = fields.StringField(required=True)
    dispense_volume = fields.FloatField(min_value=0, required=True)
    dispense_unit = fields.StringField(choices=["L", "mL", "uL"], default="mL")

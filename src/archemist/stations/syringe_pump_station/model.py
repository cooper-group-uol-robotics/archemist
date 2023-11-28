from archemist.core.models.station_op_model import StationSampleOpModel
from mongoengine import fields

class SyringePumpDispenseVolumeOpModel(StationSampleOpModel):
    liquid_name = fields.StringField(required=True)
    dispense_volume = fields.FloatField(min_value=0, required=True)
    dispense_unit = fields.StringField(choices=["L", "mL", "uL"], default="mL")
    dispense_rate = fields.FloatField(min_value=0, required=True)
    rate_unit = fields.StringField(choices=["mL/minute", "mL/second"], default="mL/minute")

class SyringePumpDispenseRateOpModel(StationSampleOpModel):
    liquid_name = fields.StringField(required=True)
    dispense_rate = fields.FloatField(min_value=0, required=True)
    rate_unit = fields.StringField(choices=["mL/minute", "mL/second"], default="mL/minute")

class SyringePumpFinishDispensingOpModel(StationSampleOpModel):
    liquid_name = fields.StringField(required=True)

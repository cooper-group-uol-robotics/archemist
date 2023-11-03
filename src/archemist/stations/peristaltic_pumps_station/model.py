from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationSampleOpDescriptorModel
from mongoengine import fields

class PeristalticPumpsStationModel(StationModel):
    liquid_pump_map = fields.MapField(fields.IntField())

class PPLiquidDispenseOpModel(StationSampleOpDescriptorModel):
    liquid_name = fields.StringField(required=True)
    dispense_volume = fields.FloatField(min_value=0, required=True)
    dispense_unit = fields.StringField(choices=["L", "mL", "uL"], default="mL")
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from mongoengine import fields

class PeristalticPumpStationModel(StationModel):
    pump_liquid_map = fields.DictField(required=True)

class PeristalticPumpOpDescriptorModel(StationOpDescriptorModel):
    liquid_name = fields.StringField(required=True)
    dispense_volume = fields.FloatField(min_value=0, required=True)
    actual_dispensed_volume = fields.FloatField(min_value=0)
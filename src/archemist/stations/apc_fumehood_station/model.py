from mongoengine import fields, EmbeddedDocument
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationSampleOpModel


class APCCartridgeModel(EmbeddedDocument):
    hotel_index = fields.IntField(required=True)
    associated_solid = fields.StringField(required=True)
    depleted = fields.BooleanField(default=False)

class APCFumehoodStationModel(StationModel):
    sash_open = fields.BooleanField(default=False)
    cartridges = fields.EmbeddedDocumentListField(APCCartridgeModel, default=[])
    loaded_cartridge_index = fields.IntField(min_value=0,null=True)

class APCDispenseSolidOpModel(StationSampleOpModel):
    solid_name = fields.StringField(required=True)
    dispense_mass = fields.FloatField(min_value=0, required=True)
    dispense_unit = fields.StringField(choices=["g", "mg", "ug"], default="g")
    
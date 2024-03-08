from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import (StationOpModel,
                                                    StationSampleOpModel)
from mongoengine import EmbeddedDocument, fields


class QuantosCartridgeModel(EmbeddedDocument):
    hotel_index = fields.IntField(required=True)
    associated_solid = fields.StringField(required=True)
    remaining_dosages = fields.IntField(min_value=0, default=100)
    blocked = fields.BooleanField(default=False)


class QuantosSolidDispenserQS2Model(StationModel):
    carousel_pos = fields.IntField(min_value=1, max_value=20, default=1)
    cartridges = fields.EmbeddedDocumentListField(
        QuantosCartridgeModel, default=[])
    loaded_cartridge_index = fields.IntField(min_value=0, null=True)
    door_open = fields.BooleanField(default=False)


class QuantosDispenseOpModel(StationSampleOpModel):
    solid_name = fields.StringField(required=True)
    dispense_mass = fields.FloatField(min_value=0, required=True)
    dispense_unit = fields.StringField(choices=["g", "mg", "ug"], default="g")


class QuantosMoveCarouselOpModel(StationOpModel):
    target_pos = fields.IntField(min_value=1, max_value=20, default=1)

from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.models.material_model import SolidMaterialModel
from mongoengine import EmbeddedDocument, fields
from enum import Enum, auto

class QuantosStatus(Enum):
    DOORS_OPEN = auto()
    DOORS_CLOSED = auto()

class QuantosCatridgeModel(EmbeddedDocument):
    exp_id = fields.IntField()
    associated_solid= fields.ReferenceField(SolidMaterialModel,required=True)
    remaining_dosages = fields.IntField(min_value=0, default=100)
    blocked = fields.BooleanField(default=False)
    hotel_index = fields.IntField(required=True)

class QuantosSolidDispenserQS2Model(StationModel):
    carousel_pos = fields.IntField(min_value=1, max_value=20, default=1)
    catridges = fields.EmbeddedDocumentListField(QuantosCatridgeModel, default=[])
    loaded_ctridge_id = fields.IntField(min_value=0, null=True)
    machine_status = fields.EnumField(QuantosStatus, default=None)

class QuantosDispenseOpDescriptorModel(StationOpDescriptorModel):
    solid_name = fields.StringField(required=True)
    dispense_mass = fields.FloatField(min_value=0, required=True)
    actual_dispensed_mass = fields.FloatField(min_value=0)

class MoveCarouselOpDescriptorModel(StationOpDescriptorModel):
    carousel_pos = fields.IntField(min_value=1, max_value=20, default=1)
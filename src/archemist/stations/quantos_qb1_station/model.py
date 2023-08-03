from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.models.material_model import SolidMaterialModel
from mongoengine import EmbeddedDocument, fields
from enum import Enum, auto

class QuantosStatus(Enum):
    """Enumerated value which conveys status of the Quantos Doors"""
    
    #TODO supoort front and side doors seperately 
    

    DOORS_OPEN = auto()
    DOORS_CLOSED = auto()

class QuantosCartridgeModel(EmbeddedDocument):
    """Model for a quantos cartridge, embedded within the QuantosDispenserQB1Model"""

    exp_id = fields.IntField()
    associated_solid= fields.ReferenceField(SolidMaterialModel,required=True)
    remaining_dosages = fields.IntField(min_value=0, default=100)
    remaining_quantity = fields.FloatField(default= 0.0)
    hotel_index = fields.IntField(required=True)


class QuantosSolidDispenserQB1Model(StationModel):
    """ Model for the quantos dispenser station """
    carousel_pos = fields.IntField(min_value=1, max_value=31, default=1) # max value of 31 to accomodate for no carousel 
    cartridges = fields.EmbeddedDocumentListField(QuantosCartridgeModel, default=[])
    loaded_ctridge_id = fields.IntField(min_value=0, null=True)
    door_status = fields.EnumField(QuantosStatus, default =None)

class DispenseOpDescriptorModel(StationOpDescriptorModel):
    """Database model for a dispensing action"""
    solid_name = fields.StringField(required=True)
    target_mass = fields.FloatField(min_value=0., required=True)
    tolerance = fields.FloatField(min_value=0.1, max_value=40)
    actual_dispensed_mass = fields.FloatField(min_value=0, default= 0)
    

class MoveCarouselOpDescriptorModel(StationOpDescriptorModel):
    """Database model for a carousel move action"""
    carousel_pos = fields.IntField(min_value=0, max_value=31, default=1)
from mongoengine import fields, EmbeddedDocument
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpModel, StationSampleOpModel
from enum import Enum, auto

class MTSynthesisPhase(Enum):
    REACTION = auto()
    FILTRATION = auto()
    CLEANING = auto()

class OptiMaxMode(Enum):
    HEATING = auto()
    STIRRING = auto()
    HEATING_STIRRING = auto()
    SAMPLING = auto()
    DRAINING = auto()
    
class SynthesisCartridgeModel(EmbeddedDocument):
    hotel_index = fields.IntField(required=True)
    associated_solid= fields.StringField(required=True)
    depleted = fields.BooleanField(default=False)


class MTSynthesisStationModel(StationModel):
    synthesis_phase = fields.EnumField(MTSynthesisPhase, default=MTSynthesisPhase.REACTION)
    optimax_mode = fields.EnumField(OptiMaxMode, null=True)
    
    
    cartridges = fields.EmbeddedDocumentListField(SynthesisCartridgeModel, default=[])
    loaded_cartridge_index = fields.IntField(min_value=0, null=True)
   
    horizontal_doors_open = fields.BooleanField(default=False)
    vertical_doors_open = fields.BooleanField(default=False)

    num_sampling_vials = fields.IntField(min_value=0, required=True)

    set_reaction_temperature = fields.IntField(min_value=-20, max_value=140, null=True)
    set_stirring_speed = fields.IntField(min_value=0, max_value=1000, null=True)

class MTSynthLoadCartridgeOpModel(StationOpModel):
    cartridge_index = fields.IntField(required=True)

class MTSynthDispenseOpModel(StationSampleOpModel):
    solid_name = fields.StringField(required=True)
    dispense_mass = fields.FloatField(min_value=0, required=True)
    dispense_unit = fields.StringField(choices=["g", "mg", "ug"], default="g")

class MTSynthLiquidDispenseOpModel(StationSampleOpModel):
    liquid_name = fields.StringField(required=True)
    dispense_volume = fields.FloatField(min_value=0, required=True)
    dispense_unit = fields.StringField(choices=["L", "mL", "uL"], default="mL")

class MTSynthReactAndWaitOpModel(StationSampleOpModel):
    target_temperature = fields.IntField(min_value=-20, max_value=140, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1000, null=True)
    wait_duration = fields.IntField(null=True)
    time_unit = fields.StringField(choices=["second", "minute", "hour"], null=True)

class MTSyntReactAndSampleOpModel(StationSampleOpModel):
    target_temperature = fields.IntField(min_value=-20, max_value=140, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1000, null=True)

class MTSynthFiltrationDrainOpModel(StationSampleOpModel):
    duration = fields.IntField(min_value=0, required=True)
    time_unit = fields.StringField(choices=["second", "minute", "hour"], default="second")

class MTSynthFiltrationVacuumOpModel(StationSampleOpModel):
    duration = fields.IntField(min_value=0, required=True)
    time_unit = fields.StringField(choices=["second", "minute", "hour"], default="second")

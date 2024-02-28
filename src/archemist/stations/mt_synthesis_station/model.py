from mongoengine import fields, EmbeddedDocument
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpModel, StationSampleOpModel
from enum import Enum, auto

class OptiMaxMode(Enum):
    HEATING = auto()
    STIRRING = auto()
    HEATING_STIRRING = auto()
    SAMPLING = auto()
    DRAINING = auto()

class MTSynthesisStationModel(StationModel):
    optimax_mode = fields.EnumField(OptiMaxMode, null=True)
    optimax_valve_open = fields.BooleanField(default=False)
    num_sampling_vials = fields.IntField(min_value=0, required=True)
    set_reaction_temperature = fields.IntField(min_value=-20, max_value=140, null=True)
    set_stirring_speed = fields.IntField(min_value=0, max_value=1000, null=True)

class MTSynthHeatStirOpModel(StationSampleOpModel):
    target_temperature = fields.IntField(min_value=-20, max_value=140, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1000, null=True)
    wait_duration = fields.IntField(null=True)
    time_unit = fields.StringField(choices=["second", "minute", "hour"], null=True)
    stir_duration = fields.IntField(null=True)

class MTSynthSampleOpModel(StationSampleOpModel):
    target_temperature = fields.IntField(min_value=-20, max_value=140, null=True)
    target_stirring_speed = fields.IntField(min_value=0, max_value=1000, null=True)
    dilution = fields.IntField(min_value=80, max_value=250, null=True)

class MTSynthCustomOpenCloseReactionValveOpModel(StationOpModel):
    steps = fields.IntField(required=True)


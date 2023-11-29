from mongoengine import Document, fields
from archemist.core.util.enums import StationState, OpState
from archemist.core.models.station_op_model import StationOpModel
from archemist.core.models.robot_op_model import RobotOpModel
from archemist.core.models.station_process_model import StationProcessModel
from archemist.core.util.location import LocationModel

class StationModel(Document):
    ''' internal '''
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)

    ''' general '''
    exp_id = fields.IntField(required=True)
    state = fields.EnumField(StationState, default=StationState.INACTIVE)
    location = fields.EmbeddedDocumentField(LocationModel)
    selected_handler = fields.StringField(required=True)

    ''' materials '''
    liquids_dict = fields.DictField(default={})
    solids_dict = fields.DictField(default={})
    
    ''' batch capacity '''
    total_lot_capacity = fields.IntField(min_value=1, default=1)
    
    ''' processes '''
    requested_ext_procs = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])
    queued_procs = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])
    running_procs = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])
    procs_history = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])
    
    ''' lots '''
    lot_slots = fields.DictField(default={})
    
    ''' robot ops '''
    requested_robot_ops = fields.ListField(fields.ReferenceField(RobotOpModel),default=[])
    
    ''' station ops '''
    requested_ext_ops = fields.ListField(fields.ReferenceField(StationOpModel), default=[])
    queued_ops = fields.ListField(fields.ReferenceField(StationOpModel),default=[])
    assigned_op = fields.ReferenceField(StationOpModel, null=True)
    assigned_op_state = fields.EnumField(OpState,default=OpState.INVALID)
    ops_history = fields.ListField(fields.ReferenceField(StationOpModel),default=[])

    meta = {'collection': 'stations', 'db_alias': 'archemist_state', 'allow_inheritance': True}
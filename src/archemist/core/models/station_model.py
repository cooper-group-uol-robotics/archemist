from mongoengine import Document, fields
from archemist.core.util.enums import StationState, OpState
from archemist.core.models.lot_model import LotModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from archemist.core.models.station_process_model import StationProcessModel
from archemist.core.models.material_model import LiquidMaterialModel, SolidMaterialModel

class StationModel(Document):
    ''' internal '''
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)

    ''' general '''
    exp_id = fields.IntField(required=True)
    state = fields.EnumField(StationState, default=StationState.INACTIVE)
    location = fields.DictField()
    selected_handler = fields.StringField(required=True)

    ''' materials '''
    liquids = fields.ListField(fields.ReferenceField(LiquidMaterialModel), default=[])
    solids = fields.ListField(fields.ReferenceField(SolidMaterialModel), default=[])
    
    ''' batch capacity '''
    total_batch_capacity = fields.IntField(min_value=1, default=1)
    process_batch_capacity = fields.IntField(min_value=1, default=1)
    
    ''' processes '''
    requested_ext_procs = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])
    queued_procs = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])
    running_procs_slots = fields.DictField(default={})
    procs_history = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])
    
    ''' lots '''
    assigned_lots = fields.ListField(fields.ReferenceField(LotModel), default=[])
    processed_lots = fields.ListField(fields.ReferenceField(LotModel), default=[])
    
    ''' robot ops '''
    requested_robot_ops = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel),default=[])
    
    ''' station ops '''
    queued_ops = fields.ListField(fields.ReferenceField(StationOpDescriptorModel),default=[])
    assigned_op = fields.ReferenceField(StationOpDescriptorModel, null=True)
    assigned_op_state = fields.EnumField(OpState,default=OpState.INVALID)
    ops_history = fields.ListField(fields.ReferenceField(StationOpDescriptorModel),default=[])

    meta = {'collection': 'stations', 'db_alias': 'archemist_state', 'allow_inheritance': True}
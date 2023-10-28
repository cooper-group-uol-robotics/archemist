from mongoengine import Document, fields
from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from archemist.core.models.lot_model import LotModel
from archemist.core.models.batch_model import BatchModel
from archemist.core.util.enums import RobotState, OpState, MobileRobotMode

class RobotModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    selected_handler = fields.StringField(required=True)
    state = fields.EnumField(RobotState, default=RobotState.INACTIVE)
    
    location = fields.DictField()
    attending_to = fields.ObjectIdField(null=True)
    
    queued_ops = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel),default=[])
    assigned_op = fields.ReferenceField(RobotOpDescriptorModel, null=True)
    assigned_op_state = fields.EnumField(OpState,default=OpState.INVALID)
    ops_history = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel), default=[])

    meta = {'collection': 'robots', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class MobileRobotModel(RobotModel):
    operational_mode = fields.EnumField(MobileRobotMode, default=MobileRobotMode.OPERATIONAL)
    total_lot_capacity = fields.IntField(min_value=1, default=1)
    consigned_lots = fields.ListField(fields.ReferenceField(LotModel),default=[])
    onboard_capacity = fields.IntField(min_value=1, default=1)
    onboard_batches_slots = fields.DictField(default={})
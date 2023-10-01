from mongoengine import Document, fields
from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from archemist.core.models.station_process_model import StationProcessModel
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.lot_model import LotModel
from archemist.core.models.recipe_model import RecipeModel

class WorkflowStateModel(Document):
    workflow_name = fields.StringField(required=True)
    
    robot_ops_queue = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel),default=[])
    proc_requests_queue = fields.ListField(fields.ReferenceField(StationProcessModel),default=[])
    lots_buffer = fields.ListField(fields.ReferenceField(LotModel), default=[])

    meta = {'collection': 'state', 'db_alias': 'archemist_state'}

class InputStateModel(Document):
    location = fields.DictField(default={})
    samples_per_batch = fields.IntField(min_value=1, default=1)
    batches_per_lot = fields.IntField(min_value=1, default=1)
    total_lot_capacity = fields.IntField(min_value=1, default=1)
    
    batches_queue = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    recipes_queue = fields.ListField(fields.ReferenceField(RecipeModel), default=[])
    requested_robot_ops = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel),default=[])

    lot_slots = fields.DictField(default={})
    proc_slots = fields.DictField(default={})

    meta = {'collection': 'state', 'db_alias': 'archemist_state'}

class OutputStateModel(Document):
    location = fields.DictField(default={})
    total_lot_capacity = fields.IntField(min_value=1, default=1)

    requested_robot_ops = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel),default=[])
    
    lot_slots = fields.DictField(default={})
    proc_slots = fields.DictField(default={})

    meta = {'collection': 'state', 'db_alias': 'archemist_state'}
from mongoengine import Document, fields
from archemist.core.models.robot_op_model import RobotOpModel
from archemist.core.models.station_process_model import StationProcessModel
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.lot_model import LotModel
from archemist.core.models.recipe_model import RecipeModel
from archemist.core.util.location import LocationModel

class WorkflowStateModel(Document):
    _type = fields.StringField(default="WorkflowStateModel")
    workflow_name = fields.StringField(required=True)
    
    robot_ops_queue = fields.ListField(fields.ReferenceField(RobotOpModel),default=[])
    proc_requests_queue = fields.ListField(fields.ReferenceField(StationProcessModel),default=[])
    lots_buffer = fields.ListField(fields.ReferenceField(LotModel), default=[])

    meta = {'collection': 'state', 'db_alias': 'archemist_state'}

class InputStateModel(Document):
    _type = fields.StringField(default="InputStateModel")
    location = fields.EmbeddedDocumentField(LocationModel)
    samples_per_batch = fields.IntField(min_value=1, default=1)
    batches_per_lot = fields.IntField(min_value=1, default=1)
    total_lot_capacity = fields.IntField(min_value=1, default=1)
    lot_input_process = fields.DictField(null=True)
    
    batches_queue = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    recipes_queue = fields.ListField(fields.ReferenceField(RecipeModel), default=[])
    requested_robot_ops = fields.ListField(fields.ReferenceField(RobotOpModel),default=[])

    lot_slots = fields.DictField(default={})
    proc_slots = fields.DictField(default={})

    procs_history = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])

    meta = {'collection': 'state', 'db_alias': 'archemist_state'}

class OutputStateModel(Document):
    _type = fields.StringField(default="OutputStateModel")
    location = fields.EmbeddedDocumentField(LocationModel)
    total_lot_capacity = fields.IntField(min_value=1, default=1)
    lot_output_process = fields.DictField(null=True)
    lots_need_manual_removal = fields.BooleanField(required=True)

    requested_robot_ops = fields.ListField(fields.ReferenceField(RobotOpModel),default=[])
    
    lot_slots = fields.DictField(default={})
    proc_slots = fields.DictField(default={})

    procs_history = fields.ListField(fields.ReferenceField(StationProcessModel), default=[])

    meta = {'collection': 'state', 'db_alias': 'archemist_state'}
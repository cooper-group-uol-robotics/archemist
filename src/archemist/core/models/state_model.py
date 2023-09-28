from mongoengine import Document, fields
from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from archemist.core.models.station_process_model import StationProcessModel
from archemist.core.models.lot_model import LotModel
from archemist.core.models.recipe_model import RecipeModel

class StateModel(Document):
    workflow_name = fields.StringField(required=True)
    
    robot_ops_queue = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel),default=[])
    proc_requests_queue = fields.ListField(fields.ReferenceField(StationProcessModel),default=[])
    recipes_queue = fields.ListField(fields.ReferenceField(RecipeModel), default=[])

    lots_buffer = fields.ListField(fields.ReferenceField(LotModel), default=[])

    meta = {'collection': 'state', 'db_alias': 'archemist_state'}
from mongoengine import Document, fields
from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.recipe_model import RecipeModel


class StateModel(Document):
    workflow_name = fields.StringField(required=True)
    samples_per_batch = fields.IntField(min_value=1)
    default_batch_input_location = fields.DictField()
    robot_ops_queue = fields.EmbeddedDocumentListField(
        RobotOpDescriptorModel, default=[]
    )
    recipes_queue = fields.ListField(fields.ReferenceField(RecipeModel), default=[])
    batches_buffer = fields.ListField(fields.ReferenceField(BatchModel), default=[])

    meta = {"collection": "state", "db_alias": "archemist_state"}

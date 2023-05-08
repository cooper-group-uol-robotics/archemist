from mongoengine import Document, fields
from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.recipe_model import RecipeModel

class OptimisationModel(Document):
    optimizer_module = fields.StringField(required=True)
    optimizer_class = fields.StringField(required=True)
    optimizer_hyperparameters = fields.DictField()


    meta = {'collection': 'optimisation', 'db_alias': 'archemist_state'}
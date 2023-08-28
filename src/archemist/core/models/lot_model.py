from mongoengine import Document, fields
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.recipe_model import RecipeModel

class LotModel(Document):
    uuid = fields.UUIDField(binary=False, required=True)
    batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    recipe = fields.ReferenceField(RecipeModel, null=True)

    meta = {'collection': 'lots', 'db_alias': 'archemist_state'}
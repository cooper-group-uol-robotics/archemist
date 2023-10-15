from mongoengine import Document, fields
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.recipe_model import RecipeModel
from archemist.core.util.enums import LotStatus

class LotModel(Document):
    status = fields.EnumField(LotStatus, default=LotStatus.CREATED)
    batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    recipe = fields.ReferenceField(RecipeModel, null=True)

    meta = {'collection': 'lots', 'db_alias': 'archemist_state'}
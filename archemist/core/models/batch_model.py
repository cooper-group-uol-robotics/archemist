# External
from mongoengine import Document, EmbeddedDocument, fields

# Core
from archemist.core.models.recipe_model import RecipeModel
from archemist.core.models.station_op_model import StationOpDescriptorModel


class SampleModel(EmbeddedDocument):
    """Description FIXME"""

    rack_index = fields.IntField(min_value=0, required=True)
    materials = fields.ListField(fields.StringField(), default=[])
    capped = fields.BooleanField(default=False)
    operation_ops = fields.EmbeddedDocumentListField(
        StationOpDescriptorModel, default=[]
    )


class BatchModel(Document):
    """Description FIXME"""

    exp_id = fields.IntField(required=True)
    num_samples = fields.IntField(min_value=1, required=True)
    location = fields.DictField()
    recipe = fields.ReferenceField(RecipeModel, null=True)
    samples = fields.EmbeddedDocumentListField(SampleModel)
    current_sample_index = fields.IntField(min_value=0, default=0)
    station_history = fields.ListField(fields.StringField(), default=[])
    meta = {"collection": "batches", "db_alias": "archemist_state"}

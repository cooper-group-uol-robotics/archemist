from mongoengine import Document, fields
from archemist.core.models.op_result_model import OpResultModel

class SampleModel(Document):
    parent_batch_id = fields.ObjectIdField(null=True)
    materials = fields.DictField(default={})
    result_ops = fields.ListField(fields.ReferenceField(OpResultModel), default=[])

    meta = {'collection': 'samples', 'db_alias': 'archemist_state'}
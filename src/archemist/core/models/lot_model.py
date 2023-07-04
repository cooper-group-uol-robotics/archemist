from mongoengine import Document, fields
from archemist.core.models.batch_model import BatchModel

class LotModel(Document):
    uuid = fields.UUIDField(binary=False, required=True)
    batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])

    meta = {'collection': 'lots', 'db_alias': 'archemist_state'}
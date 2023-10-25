from mongoengine import Document, fields
from archemist.core.models.sample_model import SampleModel
class BatchModel(Document):
    location = fields.DictField(default={})
    parent_lot_id = fields.ObjectIdField(null=True)
    samples = fields.ListField(fields.ReferenceField(SampleModel), default=[])
    station_stamps = fields.ListField(fields.StringField(), default=[])

    meta = {'collection': 'batches', 'db_alias': 'archemist_state'}


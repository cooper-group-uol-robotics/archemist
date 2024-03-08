from mongoengine import Document, fields
from archemist.core.models.sample_model import SampleModel
from archemist.core.util.location import LocationModel


class BatchModel(Document):
    location = fields.EmbeddedDocumentField(
        LocationModel, default=LocationModel())
    parent_lot_id = fields.ObjectIdField(null=True)
    samples = fields.ListField(fields.ReferenceField(SampleModel), default=[])
    station_stamps = fields.ListField(fields.StringField(), default=[])

    meta = {'collection': 'batches', 'db_alias': 'archemist_state'}

from mongoengine import connect, Document, EmbeddedDocument, fields
from archemist.core.models.station_op_model import StationOpDescriptorModel

connect('opt_test_NRA_db', host='127.0.0.1', port=27017)

class RecipeModel(Document):
    name = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    solids = fields.ListField(fields.DictField(), default=[])
    liquids = fields.ListField(fields.DictField(), default=[])
    states = fields.ListField(fields.StringField(),default=[])
    transitions = fields.ListField(fields.DictField(), default=[])
    current_state = fields.StringField(required=True)

class BatchModel(Document):
    exp_id = fields.IntField(required=True)
    num_samples = fields.IntField(min_value=1, required=True)
    location = fields.DictField()
    recipe = fields.ReferenceField(RecipeModel, null=True)
    result = fields.DictField()

    # meta = {'collection': 'batches', 'db_alias': 'archemist_state'}


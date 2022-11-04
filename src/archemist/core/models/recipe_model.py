from mongoengine import Document, fields
from archemist.core.models.station_op_model import StationOpDescriptorModel

class RecipeModel(Document):
    name = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    station_op_descriptors = fields.EmbeddedDocumentListField(StationOpDescriptorModel, default=[])
    solids = fields.ListField(fields.StringField(), default=[])
    liquids = fields.ListField(fields.StringField(), default=[])
    states = fields.ListField(fields.StringField(),default=[])
    transitions = fields.ListField(fields.DictField(), default=[])
    current_state = fields.StringField(required=True)

    meta = {'collection': 'recipes' , 'db_alias': 'archemist_state'}
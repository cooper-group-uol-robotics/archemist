from mongoengine import Document, EmbeddedDocument, fields
from archemist.core.models.station_op_model import StationOpDescriptorModel

class StateDetails(EmbeddedDocument):
    station_type = fields.StringField(required=True)
    station_id = fields.IntField(required=True)
    station_op = fields.EmbeddedDocumentField(StationOpDescriptorModel, required=True)

class RecipeModel(Document):
    name = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    state_map = fields.MapField(fields.EmbeddedDocumentField(StateDetails))
    solids = fields.ListField(fields.DictField(), default=[])
    liquids = fields.ListField(fields.DictField(), default=[])
    states = fields.ListField(fields.StringField(),default=[])
    transitions = fields.ListField(fields.DictField(), default=[])
    current_state = fields.StringField(required=True)

    meta = {'collection': 'recipes' , 'db_alias': 'archemist_state'}
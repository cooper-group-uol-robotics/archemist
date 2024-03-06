from mongoengine import Document, EmbeddedDocument, fields


class StateDetailsModel(EmbeddedDocument):
    station_type = fields.StringField(required=True)
    station_id = fields.IntField(required=True)
    station_process = fields.DictField(required=True)


class RecipeModel(Document):
    name = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    state_map = fields.MapField(fields.EmbeddedDocumentField(StateDetailsModel))
    states = fields.ListField(fields.StringField(), default=[])
    transitions = fields.ListField(fields.DictField(), default=[])
    current_state = fields.StringField(required=True)

    meta = {'collection': 'recipes', 'db_alias': 'archemist_state'}

from mongoengine import Document, fields

class RecipeModel(Document):
    name = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    station_op_descriptors = fields.ListField(fields.DictField(), default=[])
    solids = fields.ListField(fields.StringField(), default=[])
    liquids = fields.ListField(fields.StringField(), default=[])
    states = fields.ListField(fields.StringField(),default=[])
    transitions = fields.ListField(fields.DictField(), default=[])
    current_state = fields.StringField(required=True)

    meta = {'collection': 'recipes' , 'db_alias': 'archemist_state'}
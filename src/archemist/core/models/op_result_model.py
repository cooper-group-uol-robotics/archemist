from mongoengine import Document, fields

class OpResultModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    origin_op = fields.ObjectIdField(required=True)

    meta = {'collection': 'op_results', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class MaterialOpResultModel(OpResultModel):
    material_name = fields.StringField(required=True)
    material_id = fields.IntField(required=True)
    amount = fields.FloatField(min_value=0.0)
    unit = fields.StringField(required=True)

class ProcessOpResultModel(OpResultModel):
    parameters = fields.DictField(required=True)

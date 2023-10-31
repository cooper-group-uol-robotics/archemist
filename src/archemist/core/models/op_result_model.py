from mongoengine import Document, fields

class OpResultModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    origin_op = fields.ObjectIdField(required=True)

    meta = {'collection': 'op_results', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class MaterialsOpResultModel(OpResultModel):
    material_names = fields.ListField(fields.StringField(), required=True)
    amounts = fields.ListField(fields.FloatField(min_value=0.0), required=True)
    units = fields.ListField(fields.StringField(choices=["L", "mL", "uL", "g", "mg", "ug"]), required=True)

class ProcessOpResultModel(OpResultModel):
    parameters = fields.DictField(required=True)

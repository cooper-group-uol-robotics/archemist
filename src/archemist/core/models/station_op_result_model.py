from mongoengine import Document, fields


class StationOpResultModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    origin_op = fields.ObjectIdField(required=True)

    meta = {'collection': 'op_results',
            'db_alias': 'archemist_state', 'allow_inheritance': True}


class MaterialsOpResultModel(StationOpResultModel):
    material_names = fields.ListField(fields.StringField(), required=True)
    amounts = fields.ListField(fields.FloatField(min_value=0.0), required=True)
    units = fields.ListField(fields.StringField(
        choices=["L", "mL", "uL", "g", "mg", "ug"]), required=True)


class ProcessOpResultModel(StationOpResultModel):
    parameters = fields.DictField(required=True)

from mongoengine import Document, fields


class MaterialModel(Document):
    _type = fields.StringField(required=True)
    name = fields.StringField(required=True)
    belongs_to = fields.ObjectIdField(null=True)
    expiry_date = fields.DateTimeField()
    mass = fields.FloatField(min_value=0)
    mass_unit = fields.StringField(choices=["g", "mg", "ug"], default="g")
    details = fields.DictField(null=True)

    meta = {'collection': 'materials', 'db_alias': 'archemist_state', 'allow_inheritance': True}


class LiquidMaterialModel(MaterialModel):
    volume = fields.FloatField(min_value=0)
    volume_unit = fields.StringField(choices=["L", "mL", "uL"], default="mL")
    density = fields.FloatField(min_value=0)
    density_unit = fields.StringField(choices=["kg/m3", "g/cm3"], default="kg/m3")


class SolidMaterialModel(MaterialModel):
    pass

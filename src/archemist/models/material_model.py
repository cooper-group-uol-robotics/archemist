from mongoengine import Document, fields

class MaterialModel(Document):
    _type = fields.StringField(required=True)
    name = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    expiry_date = fields.DateTimeField()
    mass = fields.FloatField(min_value=0)

    meta = {'collection': 'materials', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class LiquidMaterialModel(MaterialModel):
    pump_id = fields.StringField()
    volume = fields.FloatField(min_value=0)
    density = fields.FloatField(min_value=0)

class SolidMaterialModel(MaterialModel):
    dispense_src = fields.StringField()
    cartridge_id = fields.IntField()
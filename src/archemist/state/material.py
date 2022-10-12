from datetime import date
from bson.objectid import ObjectId
from mongoengine import Document, fields

class MaterialModel(Document):
    name = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    expiry_date = fields.DateTimeField()
    mass = fields.FloatField(min_value=0)

    meta = {'collection': 'materials', 'db_alias': 'archemist_state', 'allow_inheritance': True}


class Material:
    def __init__(self, material_model: MaterialModel) -> None:
        self._model = material_model

    @property
    def model(self) -> MaterialModel:
        self._model.reload()
        return self._model
    
    @property
    def name(self) -> str:
        return self._model.name

    @property
    def id(self) -> int:
        return self._model.exp_id

    @property
    def expiry_date(self) -> date:
        return date.fromisoformat(self._model.expiry_date)

    @property
    def mass(self) -> float:
        self._model.reload('mass')
        return self._model.mass

    @mass.setter
    def mass(self, value):
        self._model.update(mass=value)

class LiquidMaterialModel(MaterialModel):
    pump_id = fields.StringField()
    volume = fields.FloatField(min_value=0)
    density = fields.FloatField(min_value=0)


class Liquid(Material):
    def __init__(self, material_model: LiquidMaterialModel) -> None:
        self._model = material_model

    @classmethod
    def from_dict(cls, liquid_document: dict):
        model = LiquidMaterialModel()
        model.name = liquid_document['name']
        model.exp_id = liquid_document['id']
        model.pump_id = liquid_document['pump_id']
        model.expiry_date = date.isoformat(liquid_document['expiry_date'])
        model.density = liquid_document['density']
        if liquid_document['unit'] in ['l','ml','ul']:
            if liquid_document['unit'] == 'l':
                unit_modifier = 1
            elif liquid_document['unit'] == 'ml':
                unit_modifier = 1000
            elif liquid_document['unit'] == 'ul':
                unit_modifier = 1000000
            model.volume = liquid_document['amount_stored']/unit_modifier
            model.mass = model.density * model.volume
        elif liquid_document['unit'] in ['g','mg','ug']:
            if liquid_document['unit'] == 'g':
                unit_modifier = 1
            elif liquid_document['unit'] == 'mg':
                unit_modifier = 1000
            elif liquid_document['unit'] == 'ug':
                unit_modifier = 1000000
            model.mass = liquid_document['amount_stored']/unit_modifier
            model.volume = model.mass/model.density
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = MaterialModel.objects.get(id=object_id)
        return cls(model)

    @property #return in g/l which is equivalent to kg/m3
    def density(self) -> float:
        return self._model.density

    @property
    def pump_id(self) -> str:
        return self._model.pump_id

    @property # return in l
    def volume(self) -> float:
        self._model.reload('volume')
        return self._model.volume

    @volume.setter
    def volume(self, new_volume):
        self._model.update(volume=new_volume)

    def __str__(self):
        return f'Liquid: {self.name}, Pump ID: {self.pump_id}, Expiry date: {self.expiry_date},\
                 Mass: {self.mass} g, Volume: {self.volume} L,\
                 Density: {self.density} g/L'

class SolidMaterialModel(MaterialModel):
    dispense_src = fields.StringField()

class Solid(Material):
    def __init__(self, material_model: SolidMaterialModel) -> None:
        self._model = material_model

    @classmethod
    def from_dict(cls, solid_document: dict):
        model = SolidMaterialModel()
        model.name = solid_document['name']
        model.exp_id = solid_document['id']
        model.dispense_src = solid_document['dispense_method']
        model.expiry_date = date.isoformat(solid_document['expiry_date'])
        if solid_document['unit'] == 'g':
            unit_modifier = 1
            model.mass = solid_document['amount_stored']/unit_modifier
        elif solid_document['unit'] == 'mg':
            unit_modifier = 1000
            model.mass = solid_document['amount_stored']/unit_modifier
        elif solid_document['unit'] == 'ug':
            unit_modifier = 1000000
            model.mass = solid_document['amount_stored']/unit_modifier
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = MaterialModel.objects.get(id=object_id)
        return cls(model)
    
    @property
    def dispense_src(self):
        return self._model.dispense_src

    def __str__(self):
        return f'Solid: {self.name}, Expiry date: {self.expiry_date},\
                 Mass: {self.mass} g, Dispense method: {self.dispense_src}'

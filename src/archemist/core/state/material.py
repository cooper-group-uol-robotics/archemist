from datetime import date
from bson.objectid import ObjectId
from archemist.core.models.material_model import MaterialModel, SolidMaterialModel, LiquidMaterialModel


class Material:
    def __init__(self, material_model: MaterialModel) -> None:
        self._model = material_model

    @staticmethod
    def _set_model_common_fields(material_dict: dict, station_model: MaterialModel):
        station_model.name = material_dict['name']
        station_model.exp_id = material_dict['id']
        station_model.expiry_date = date.isoformat(material_dict['expiry_date'])

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

class Liquid(Material):
    def __init__(self, material_model: LiquidMaterialModel) -> None:
        self._model = material_model

    @classmethod
    def from_dict(cls, liquid_doct: dict):
        model = LiquidMaterialModel()
        model._type = cls.__name__
        cls._set_model_common_fields(liquid_doct,model)
        model.pump_id = liquid_doct['pump_id']
        model.density = liquid_doct['density']
        if liquid_doct['unit'] in ['l','ml','ul']:
            if liquid_doct['unit'] == 'l':
                unit_modifier = 1
            elif liquid_doct['unit'] == 'ml':
                unit_modifier = 1000
            elif liquid_doct['unit'] == 'ul':
                unit_modifier = 1000000
            model.volume = liquid_doct['amount_stored']/unit_modifier
            model.mass = model.density * model.volume
        elif liquid_doct['unit'] in ['g','mg','ug']:
            if liquid_doct['unit'] == 'g':
                unit_modifier = 1
            elif liquid_doct['unit'] == 'mg':
                unit_modifier = 1000
            elif liquid_doct['unit'] == 'ug':
                unit_modifier = 1000000
            model.mass = liquid_doct['amount_stored']/unit_modifier
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

class Solid(Material):
    def __init__(self, material_model: SolidMaterialModel) -> None:
        self._model = material_model

    @classmethod
    def from_dict(cls, solid_doct: dict):
        model = SolidMaterialModel()
        model._type = cls.__name__
        cls._set_model_common_fields(solid_doct,model)
        model.dispense_src = solid_doct['dispense_src']
        if model.dispense_src == 'quantos':
            model.cartridge_id = solid_doct['cartridge_id']
        if solid_doct['unit'] == 'g':
            unit_modifier = 1
            model.mass = solid_doct['amount_stored']/unit_modifier
        elif solid_doct['unit'] == 'mg':
            unit_modifier = 1000
            model.mass = solid_doct['amount_stored']/unit_modifier
        elif solid_doct['unit'] == 'ug':
            unit_modifier = 1000000
            model.mass = solid_doct['amount_stored']/unit_modifier
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = MaterialModel.objects.get(id=object_id)
        return cls(model)
    
    @property
    def dispense_src(self):
        return self._model.dispense_src

    @property
    def cartridge_id(self):
        return self._model.cartridge_id

    def __str__(self):
        return f'Solid: {self.name}, Expiry date: {self.expiry_date},\
                 Mass: {self.mass} g, Dispense method: {self.dispense_src}'

from datetime import date
from bson.objectid import ObjectId
from archemist.core.models.material_model import MaterialModel, SolidMaterialModel, LiquidMaterialModel
from archemist.core.persistence.models_proxy import ModelProxy
from typing import Union, Literal, Dict, Any
from archemist.core.util.units import L, mL, uL, kg, g, mg, ug, m3, cm3, mm3
from unyt.array import unyt_quantity


class Material:
    def __init__(self, material_model: Union[MaterialModel, ModelProxy]) -> None:
        if isinstance(material_model, ModelProxy):
            self._model_proxy = material_model
        else:
            self._model_proxy = ModelProxy(material_model)

    @staticmethod
    def _set_model_common_fields(material_dict: dict, material_model: MaterialModel):
        material_model.name = material_dict['name']
        material_model.expiry_date = date.isoformat(
            material_dict['expiry_date'])
        material_model.details = material_dict.get('details')

    @property
    def object_id(self) -> ObjectId:
        return self._model_proxy.object_id

    @property
    def model(self) -> MaterialModel:
        return self._model_proxy.model

    @property
    def name(self) -> str:
        return self._model_proxy.name

    @property
    def belongs_to(self) -> ObjectId:
        return self._model_proxy.belongs_to

    @belongs_to.setter
    def belongs_to(self, new_owner: ObjectId):
        self._model_proxy.belongs_to = new_owner

    @property
    def expiry_date(self) -> date:
        return self._model_proxy.expiry_date.date()

    @property
    def mass(self) -> float:
        return self._model_proxy.mass

    @property
    def mass_unit(self) -> str:
        return self._model_proxy.mass_unit

    def increase_mass(self, quantity: float, unit: Literal["kg", "g", "mg", "ug"]):
        modified_mass = self.mass*eval(self.mass_unit) + quantity*eval(unit)
        self._update_mass(modified_mass)

    def decrease_mass(self, quantity: float, unit: Literal["kg", "g", "mg", "ug"]):
        modified_mass = self.mass*eval(self.mass_unit) - quantity*eval(unit)
        self._update_mass(modified_mass)

    def _update_mass(self, mass_quantity: unyt_quantity):
        new_mass = mass_quantity.to(self.mass_unit)
        self._model_proxy.mass = new_mass.to_value()

    @property
    def details(self) -> Dict[str, Any]:
        return self._model_proxy.details


class Liquid(Material):
    def __init__(self, material_model: Union[LiquidMaterialModel, ModelProxy]) -> None:
        super().__init__(material_model)

    @classmethod
    def from_dict(cls, liquid_dict: dict):
        model = LiquidMaterialModel()
        model._type = cls.__name__
        cls._set_model_common_fields(liquid_dict, model)
        model.density = liquid_dict['density']
        model.density_unit = liquid_dict['density_unit']
        density = model.density*eval(model.density_unit)
        if liquid_dict['unit'] in ["L", "mL", "uL"]:
            model.volume = liquid_dict['amount']
            model.volume_unit = liquid_dict['unit']
            volume = model.volume*eval(model.volume_unit)
            mass = density * volume
            mass.convert_to_units("g")
            model.mass = mass.to_value()
            model.mass_unit = "g"
        elif liquid_dict['unit'] in ["g", "mg", "ug"]:
            model.mass = liquid_dict['amount']
            model.mass_unit = liquid_dict['unit']
            mass = model.mass*eval(model.mass_unit)
            volume = mass / density
            volume.convert_to_units("mL")
            model.volume = volume.to_value()
            model.volume_unit = "mL"
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = MaterialModel.objects.get(id=object_id)
        return cls(model)

    @property
    def density(self) -> float:
        return self._model_proxy.density

    @property
    def density_unit(self) -> str:
        return self._model_proxy.density_unit

    @property
    def volume(self) -> float:
        return self._model_proxy.volume

    @property
    def volume_unit(self) -> str:
        return self._model_proxy.volume_unit

    def increase_volume(self, quantity: float, unit: Literal["L", "mL", "uL", "m3", "cm3", "mm3"]):
        modified_volume = self.volume * \
            eval(self.volume_unit) + quantity*eval(unit)
        modified_mass = self.density*eval(self.density_unit)*modified_volume
        self._update_volume(modified_volume)
        self._update_mass(modified_mass)

    def decrease_volume(self, quantity: float, unit: Literal["L", "mL", "uL", "m3", "cm3", "mm3"]):
        modified_volume = self.volume * \
            eval(self.volume_unit) - quantity*eval(unit)
        modified_mass = self.density*eval(self.density_unit)*modified_volume
        self._update_volume(modified_volume)
        self._update_mass(modified_mass)

    def increase_mass(self, quantity: float, unit: Literal["kg", "g", "mg", "ug"]):
        super().increase_mass(quantity, unit)
        new_volume = self.mass*eval(self.mass_unit) / \
            self.density*eval(self.density_unit)
        self._update_volume(new_volume)

    def decrease_mass(self, quantity: float, unit: Literal["kg", "g", "mg", "ug"]):
        super().decrease_mass(quantity, unit)
        new_volume = self.mass*eval(self.mass_unit) / \
            self.density*eval(self.density_unit)
        self._update_volume(new_volume)

    def _update_volume(self, vol_quantity: unyt_quantity):
        new_vol = vol_quantity.to(self.volume_unit)
        self._model_proxy.volume = new_vol.to_value()

    def __str__(self):
        return f'Liquid: {self.name}, Expiry date: {self.expiry_date},\
                 Mass: {self.mass} {self.mass_unit}, Volume: {self.volume} {self.volume_unit}'


class Solid(Material):
    def __init__(self, material_model: Union[SolidMaterialModel, ModelProxy]) -> None:
        super().__init__(material_model)

    @classmethod
    def from_dict(cls, solid_dict: dict):
        model = SolidMaterialModel()
        model._type = cls.__name__
        cls._set_model_common_fields(solid_dict, model)
        model.mass = solid_dict['amount']
        model.mass_unit = solid_dict['unit']
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = MaterialModel.objects.get(id=object_id)
        return cls(model)

    def __str__(self):
        return f'Solid: {self.name}, Expiry date: {self.expiry_date},\
                 Mass: {self.mass} {self.mass_unit}'

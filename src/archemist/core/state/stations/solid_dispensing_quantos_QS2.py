from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Solid, Liquid
from archemist.core.models.material_model import SolidMaterialModel
from typing import Dict, List
from archemist.core.exceptions.exception import UsingConsumedCatridgeError, QuantosCatridgeLoadedError
from mongoengine import EmbeddedDocument, fields
from datetime import datetime

class QuantosCatridgeModel(EmbeddedDocument):
    exp_id = fields.IntField()
    associated_solid= fields.ReferenceField(SolidMaterialModel,required=True)
    remaining_dosages = fields.IntField(min_value=0, default=100)
    blocked = fields.BooleanField(default=False)
    hotel_index = fields.IntField(required=True)




class QuantosCatridge:
    def __init__(self, catridge_model: QuantosCatridgeModel) -> None:
        self._model = catridge_model

    @classmethod
    def from_args(cls, id: int, solid: Solid, hotel_index: int, remaining_dosages: int):
        model = QuantosCatridgeModel()
        model.exp_id = id
        model.associated_solid = solid.model
        model.hotel_index = hotel_index
        model.remaining_dosages = remaining_dosages
        return cls(model)

    @property
    def model(self) -> QuantosCatridgeModel:
        return self._model

    @property
    def id(self) -> int:
        return self._model.exp_id

    @property
    def associated_solid(self) -> Solid:
        return Solid(self._model.associated_solid)

    @property
    def hotel_index(self) -> int:
        return self._model.hotel_index

    @property
    def consumed(self) -> bool:
        return self._model.remaining_dosages == 0

    @property
    def blocked(self) -> bool:
        return self._model.blocked

    @property
    def remaining_dosages(self) -> int:
        return self._model.remaining_dosages

    def dispense(self, dispense_amount: float):
        self.associated_solid.mass -= dispense_amount
        self._model.remaining_dosages -= 1


class QuantosSolidDispenserQS2Model(StationModel):
    carousel_pos = fields.IntField(min_value=1, max_value=20, default=1)
    catridges = fields.EmbeddedDocumentListField(QuantosCatridgeModel, default=[])
    loaded_ctridge_id = fields.IntField(min_value=0, null=True)
    doors_open = fields.BooleanField(default=False)


class QuantosSolidDispenserQS2(Station):
    def __init__(self, station_model: QuantosSolidDispenserQS2Model) -> None:
        self._model = station_model
        self._loaded_catridge_index = None

    
    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        model = QuantosSolidDispenserQS2Model()
        cls._set_model_common_fields(station_dict, model)
        model._module = cls.__module__
        parameters = station_dict['parameters']
        for cat in parameters['catridges']:
            for solid in solids:
                if solid.cartridge_id is not None and solid.cartridge_id == cat['id']:
                    catridge = QuantosCatridge.from_args(id=cat['id'], solid=solid, 
                                   hotel_index=cat['hotel_index'],
                                   remaining_dosages=cat['remaining_dosages'])
                    model.catridges.append(catridge.model)
        model.save()
        return cls(model)

    @property
    def carousel_pos(self) -> int:
        self._model.reload('carousel_pos')
        return self._model.carousel_pos

    @carousel_pos.setter
    def carousel_pos(self, new_pos: int):
        self._model.update(carousel_pos=new_pos)

    @property
    def current_catridge(self) -> QuantosCatridge:
        self._model.reload('loaded_ctridge_id')
        loaded_ctridge_id = self._model.loaded_ctridge_id
        if loaded_ctridge_id is not None:
            i = 0
            for catridge_model in self._model.catridges:
                if catridge_model.exp_id == loaded_ctridge_id:
                    self._loaded_catridge_index = i
                    return QuantosCatridge(catridge_model)
                i += 1

    @property
    def doors_open(self) -> bool:
        self._model.reload('doors_open')
        return self._model.doors_open

    @doors_open.setter
    def doors_open(self, new_state: bool):
        self._model.update(doors_open=new_state)

    def get_cartridge_id(self, solid_name: str):
        for catridge_model in self._model.catridges:
            if catridge_model.associated_solid.name == solid_name:
                return catridge_model.exp_id

    def load_catridge(self, cartridge_id: int):
        self._model.reload('loaded_ctridge_id')
        if self._model.loaded_ctridge_id is None:
            self._model.update(loaded_ctridge_id=cartridge_id)
        else:
            raise QuantosCatridgeLoadedError()

    def unload_current_catridge(self):
        self._model.reload('loaded_ctridge_id')
        if self._model.loaded_ctridge_id is not None:
            self._model.update(unset__loaded_ctridge_id=True)
        else:
            print('unloading catridge when no catridge is loaded!!!')

    def dispense(self, dispense_amount: float):
        current_catridge = self.current_catridge
        if not current_catridge.consumed and not current_catridge.blocked:
            current_catridge.dispense(dispense_amount)
            self._model.update(**{f'catridges__{self._loaded_catridge_index}':current_catridge.model})
            if current_catridge.consumed:
                self._log_station(f'the catridge {current_catridge.id} does not have any dosages left anymore. Please replace it.')
        else:
            self._log_station(f'Current catridge {current_catridge.id} is either consumed or blocked. Cannot Dispense solid!!!')

    def complete_assigned_station_op(self, success: bool, **kwargs):
        current_op = self.get_assigned_station_op()
        if isinstance(current_op, QuantosDispenseOpDescriptor):
            if success and current_op.solid_name == self.current_catridge.associated_solid.name:
                if 'actual_dispensed_mass' in kwargs:
                    self.dispense(kwargs['actual_dispensed_mass'])
                else:
                    self.dispense(current_op.dispense_mass)
        super().complete_assigned_station_op(success, **kwargs)

''' ==== Station Operation Descriptors ==== '''
class OpenDoorOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)


class CloseDoorOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class MoveCarouselOpDescriptorModel(StationOpDescriptorModel):
    carousel_pos = fields.IntField(min_value=1, default=1)


class MoveCarouselOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: MoveCarouselOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = MoveCarouselOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        model.carousel_pos = kwargs['carousel_pos']
        return cls(model)

    @property
    def carousel_pos(self) -> int:
        return self._model.carousel_pos


class QuantosDispenseOpDescriptorModel(StationOpDescriptorModel):
    solid_name = fields.StringField(required=True)
    dispense_mass = fields.FloatField(min_value=0, required=True)
    actual_dispensed_mass = fields.FloatField(min_value=0)


class QuantosDispenseOpDescriptor(StationOpDescriptor):
    def __init__(self, op_model: QuantosDispenseOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = QuantosDispenseOpDescriptorModel()
        model._type = cls.__name__
        model._module = cls.__module__
        model.solid_name = kwargs['solid_name']
        model.dispense_mass = kwargs['dispense_mass']
        return cls(model)

    @property
    def solid_name(self) -> str:
        return self._model.solid_name

    @property
    def dispense_mass(self) -> float:
        return self._model.dispense_mass

    @property
    def actual_dispensed_mass(self) -> float:
        if self._model.has_result and self._model.was_successful:
            return self._model.actual_dispensed_mass

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        if 'actual_dispensed_mass' in kwargs:
            self._model.actual_dispensed_mass = kwargs['actual_dispensed_mass']
        else:
            print('missing actual dispensed mass!!')


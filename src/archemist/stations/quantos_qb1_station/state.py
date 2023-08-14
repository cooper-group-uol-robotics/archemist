from .model import (QuantosCartridgeModel, QuantosSolidDispenserQB1Model,
                    DispenseOpDescriptorModel, MoveCarouselOpDescriptorModel,
                    QuantosStatus)
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOpDescriptor
from archemist.core.state.material import Solid, Liquid
from typing import Dict, List
from archemist.core.exceptions.exception import QuantosCartridgeLoadedError, QuantosCartridgeUnloadedError
from datetime import datetime
import re

''' ==== Station Description ==== '''
class QuantosCartridge:
    #Wrapper around the embedded QuantosCartridge model

    def __init__(self, cartridge_model: QuantosCartridgeModel) -> None:
        self._model = cartridge_model

    @classmethod
    def from_args(cls, id: int, solid: Solid, mass_unit: str, hotel_index: int, remaining_dosages: int, remaining_quantity: float) :
        #Construct a quantos cartridge model from arguments
        model = QuantosCartridgeModel()
        model.exp_id = id
        model.associated_solid = solid.model
        model.hotel_index = hotel_index
        model.mass_unit = mass_unit
        model.remaining_dosages = remaining_dosages
        model.remaining_quantity = remaining_quantity
        return cls(model)
    

    @property
    def model(self) -> QuantosCartridgeModel:
        """Returns the model as a QuantosCartridgeModel object"""
        return self._model

    @property
    def mass_unit(self) -> str:
        """Returns the mass unit of the remaining quantity in the cartridge"""
        return self._model.mass_unit

    @property
    def id(self) -> int:
        """Returns the cartridge id"""
        return self._model.exp_id

    @property
    def associated_solid(self) -> Solid:
        """Returns the solid inside the cartridge as a Solid object"""
        return Solid(self._model.associated_solid)

    @property
    def hotel_index(self) -> int:
        """Returns the hotel index of the cartridge"""
        return self._model.hotel_index

    @property
    def consumed(self) -> bool:
        """If 0 doses remaining returns true"""
        return self._model.remaining_dosages == 0


    @property
    def remaining_dosages(self) -> int:
        """Returns the remaining doses"""
        return self._model.remaining_dosages

    @remaining_dosages.setter
    def remaining_dosages(self, doses:int):
        self._model.remaining_dosages = self.remaining_dosages - doses

    @property
    def remaining_quantity(self) -> float:
        """Returns the remaining quantity"""
        return self._model.remaining_quantity

    @remaining_quantity.setter
    def remaining_quantity(self, quantity:float):
        self._model.remaining_quantity = self._model.remaining_quantity - quantity


    def dispense(self, dispense_amount: float):
        """Checks the unit of the dispense step ( which must be mg) against the unit of the 
        cartridge remaining quantity (g / mg) and adjusts the values of remaining quantity,
        associated solid mass (g) and remaining amount.
        """
      

        self.associated_solid.mass -= dispense_amount/1000
        if self.mass_unit == 'mg':
            self.remaining_quantity -= dispense_amount
        elif self.mass_unit == 'g':
            self.remaining_quantity -= dispense_amount/1000
        else: 
            print("Cartridge remaining quantities must be in g or mg")
        
        
        self._model.remaining_dosages -= 1


class QuantosSolidDispenserQB1(Station):
    #wrapper around the QuantosSolidDispenserQB1 station model
    def __init__(self, station_model: QuantosSolidDispenserQB1Model) -> None:
        self._model = station_model
        self._loaded_cartridge_index = None

    
    @classmethod
    def from_dict(cls, station_dict: Dict, liquids: List[Liquid], solids: List[Solid]):
        """Constructs a wrapper around the QuantosSolidDispenserQB1Model

        station_dict: parameters that set up the station as specified in the model
        solids/liquids: specified in the recipe
        """

        #Standard constructors same for every station class
        model = QuantosSolidDispenserQB1Model()
        cls._set_model_common_fields(station_dict, model)
        model._module = cls.__module__
        
        parameters = station_dict['parameters']
        
        #Checks if station_dict['parameters'['cartridges']['id']] == solid.cartridge_id - if it is, creates a Quantos Cartridge object 
        for cat in parameters['cartridges']:
 #
            for solid in solids:
                
                if solid.cartridge_id is not None and solid.cartridge_id == cat['id']:
                   
                    cartridge = QuantosCartridge.from_args(id=cat['id'], solid=solid, 
                                   hotel_index=int(cat['hotel_index']),
                                   remaining_dosages=int(cat['remaining_dosages']),
                                   remaining_quantity=float(cat['remaining_quantity']),
                                   mass_unit=str(cat["mass_unit"]))
                    
                    model.cartridges.append(cartridge.model)
        
        
        model.save()
        return cls(model)

    @property
    def carousel_pos(self) -> int:
        """ Returns carousel_position """
        self._model.reload('carousel_pos')
        return self._model.carousel_pos

    @carousel_pos.setter
    def carousel_pos(self, new_pos: int):
        """Sets carousel_position"""
        self._model.update(carousel_pos=new_pos)

    @property 
    def current_cartridge(self) -> QuantosCartridge:
        """Checks if the loaded_ctridge_id matches an exp_id from a cartridge in the database, if it does returns it as a QuantosCartridge
            Also sets a _loaded_cartridge_index variable to an integer value
        """

        self._model.reload('loaded_ctridge_id') 
        loaded_ctridge_id = self._model.loaded_ctridge_id
        if loaded_ctridge_id is not None:
            i = 0
            for cartridge_model in self._model.cartridges:
                if cartridge_model.exp_id == loaded_ctridge_id:
                    self._loaded_cartridge_index = i
                    return QuantosCartridge(cartridge_model)
                i += 1

    @property
    def status(self) -> QuantosStatus:
        "Gets door status"
        self._model.reload('door_status')
        return self._model.door_status

    @status.setter
    def status(self, new_state: QuantosStatus):
        "Sets door status"
        self._model.update(door_status=new_state)

    def get_cartridge_id(self, solid_name: str):
        "Gets cartridge_id associated with a paticular solid name"

        for cartridge_model in self._model.cartridges:
            
            if cartridge_model.associated_solid.name == solid_name:
                return cartridge_model.exp_id

    def load_cartridge(self, cartridge_id: int):
        """If a cartridge is not loaded, loads a cartridge and sets the loaded_ctridge_id variable"""
        
        self._model.reload('loaded_ctridge_id')
        if self._model.loaded_ctridge_id is None:
            self._model.update(loaded_ctridge_id=cartridge_id)

        elif self._model.loaded_ctridge_id == cartridge_id:
            print(f"Cartridge with {cartridge_id} already loaded")

        else:
            raise QuantosCartridgeLoadedError()

    def unload_current_cartridge(self):
        """Unloads the cartridge if one is loaded"""
        
        self._model.reload('loaded_ctridge_id')
        if self._model.loaded_ctridge_id is not None:
            self._model.update(unset__loaded_ctridge_id=True)
        else:
            raise QuantosCartridgeUnloadedError

    def update_cartridge_dispense(self, dispense_amount: float):
        """ Updates the dispense amounts in the loaded cartridge, logs if cartridges are consumed."""
        current_cartridge = self.current_cartridge

    
        if not current_cartridge.consumed:
            current_cartridge.dispense(dispense_amount)
            self._model.update(**{f'cartridges__{self._loaded_cartridge_index}':current_cartridge.model})
        
        if current_cartridge.consumed:
                    self._log_station(f'the cartridge {current_cartridge.id} does not have any dosages left anymore. Please replace it.')



    def update_assigned_op(self):
        super().update_assigned_op()
        
      

    def complete_assigned_station_op(self, success: bool, **kwargs):
        """checks the StationOpDescriptor object and updates the database model accordingly"""
     
        current_op = self.get_assigned_station_op() #gets the operation type as a StationOPDescriptor object

        
        
        if isinstance(current_op, DispenseOpDescriptor):
           
            
            actual_dispensed_mass = current_op._read_dispensed_mass(**kwargs)
           
            
            if success and current_op.solid_name == self.current_cartridge.associated_solid.name:                  
                if current_op.actual_dispensed_mass != 0:
                    self.update_cartridge_dispense(actual_dispensed_mass)
                else:
                    print('missing actual dispensed mass or dispensed nothing')
        
        elif isinstance(current_op, OpenDoorOpDescriptor):
            self.status = QuantosStatus.DOORS_OPEN
        elif isinstance(current_op, CloseDoorOpDescriptor):
            self.status = QuantosStatus.DOORS_CLOSED
        elif isinstance(current_op, MoveCarouselOpDescriptor):
            self.carousel_pos = current_op.carousel_pos
        super().complete_assigned_station_op(success, **kwargs)

''' ==== Station Operation Descriptors ==== '''
class OpenDoorOpDescriptor(StationOpDescriptor):
    """Operation descriptor for opening the station door """
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=QuantosSolidDispenserQB1.__name__, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class CloseDoorOpDescriptor(StationOpDescriptor):
    """Operation descriptor for closing the station door """
    def __init__(self, op_model: StationOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=QuantosSolidDispenserQB1.__name__, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)

class UnlockCartridgeOpDescriptor(StationOpDescriptor):
    """Operation descriptor for unlocking the cartridge after dispense"""
    def __init__(self, op_model: StationOpDescriptorModel) -> None:
        self._model = op_model


    @classmethod
    def from_args(cls, **kwargs):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=QuantosSolidDispenserQB1.__name__,**kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)


class MoveCarouselOpDescriptor(StationOpDescriptor):
    "Operation descriptor for moving the carousel"
    def __init__(self, op_model: MoveCarouselOpDescriptorModel):
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = MoveCarouselOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=QuantosSolidDispenserQB1.__name__, **kwargs) 
        model._type = cls.__name__
        model._module = cls.__module__
        model.carousel_pos = kwargs['carousel_pos']
        return cls(model)

    @property
    def carousel_pos(self) -> int: 
        return self._model.carousel_pos

class DispenseOpDescriptor(StationOpDescriptor):
    "Operation descriptor for the dispense"
    def __init__(self, op_model: DispenseOpDescriptorModel) -> None:
        self._model = op_model

    @classmethod
    def from_args(cls, **kwargs):
        model = DispenseOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station=QuantosSolidDispenserQB1.__name__, **kwargs)
        model._type = cls.__name__
        model._module = cls.__module__
        model.solid_name = kwargs['solid_name']
        model.target_mass = float(kwargs['target_mass'])
        model.mass_unit = str(kwargs['mass_unit'])
        model.tolerance = float(kwargs['tolerance'])
       
        
        return cls(model)

    @property
    def mass_unit(self) -> str:
        return self._model.mass_unit


    @property
    def solid_name(self) -> str:
        return self._model.solid_name

    @property
    def target_mass(self) -> float:
        return self._model.target_mass

    @property
    def tolerance(self) -> float:
        return self._model.tolerance
    
    @property
    def actual_dispensed_mass(self) -> float:
        if self._model.has_result and self._model.was_successful:
            return self._model.actual_dispensed_mass

    @actual_dispensed_mass.setter
    def actual_dispensed_mass(self, value:float):
        self._model.actual_dispensed_mass = value

    @property
    def dispense_metadata(self)-> list:
        if self._model.has_result and self._model.was_successful:
            return self._model.dispense_metadata


    @dispense_metadata.setter
    def dispense_metadata(self, data:List):
        self._model.dispense_metadata = data

    
    def complete_op(self, success: bool, **kwargs):
       
        
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()
        
        output_values = kwargs["output"]
        self.dispense_metadata = output_values
    
    def _read_dispensed_mass(self, **kwargs):
        
        output_values = kwargs["output"]
       
        dispensed_mass = re.findall("\d*\.\d*", output_values[1])
        
        if dispensed_mass:
            mass_as_float = float(dispensed_mass[0])
            self.actual_dispensed_mass = mass_as_float

            return mass_as_float

        else:
            print("missing actual dispensed mass")

            return 0

        


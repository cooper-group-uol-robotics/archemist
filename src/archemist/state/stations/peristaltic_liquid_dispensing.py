from transitions.core import Machine
from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from archemist.state.material import Liquid
from archemist.exceptions.exception import InvalidLiquidError
from archemist.util.location import Location


class PeristalticLiquidDispensing(Station):
    def __init__(self, id: int, location:Location, process_sm: Machine, parameters: dict, 
                 liquids: list, solids: list):
        super().__init__(id, location, process_sm)
        self._pump_liquid_map = dict()
        for liquidName, pumpId in parameters['liquid_pump_map'].items():
            for liquid in liquids:
                if liquid.pump_id == pumpId:
                    self._pump_liquid_map[pumpId] = liquid

    def get_pump_liquid_level(self, pumpId: str):
        return self._pump_liquid_map[pumpId].volume

    def get_pump_id(self, requested_liquid: Liquid):
        # assuming only single liquid per pump
        for pumpId, liquid in self._pump_liquid_map.items():
            if liquid.name == requested_liquid.name:
                return pumpId
            else:
                raise InvalidLiquidError(self.__class__.__name__)

    def get_liquid_level(self, liquid: Liquid):
        pumpId = self.get_pump_id(liquid)
        return self.get_pump_liquid_level(pumpId)

    def add_to_pump_liquid_level(self, pumpId: str, added_value: float):
        self._pump_liquid_map[pumpId].volume = self._pump_liquid_map[pumpId].volume + added_value

    def add_liquid(self, added_liquid: Liquid):
        pumpId = self.get_pump_id(added_liquid)
        self.add_to_pump_liquid_level(pumpId, added_liquid.volume)

    def reduce_pump_liquid_level(self, pumpId: str, added_value: float):
        self._pump_liquid_map[pumpId].volume = self._pump_liquid_map[pumpId].volume - added_value

    def dispense_liquid(self, dispensed_liquid: Liquid):
        pumpId = self.get_pump_id(dispensed_liquid)
        self.reduce_pump_liquid_level(pumpId, dispensed_liquid.volume)

    def set_statio_op(self, stationOp: StationOpDescriptor):
        if (stationOp.stationName == self.__class__.__name__):
            self.dispense_liquid(stationOp.liquid)
        else:
            raise ValueError

''' ==== Station Operation Descriptors ==== '''

class PeristalticPumpOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=PeristalticLiquidDispensing.__name__, output=output)
        self._liquid_name = properties['liquid']
        self._dispense_volume = properties['volume']

    @property
    def liquid_name(self):
        return self._liquid_name

    @property
    def dispense_volume(self):
        return self._dispense_volume



''' ==== Station Output Descriptors ==== '''

class PeristalticPumpOutputDescriptor(StationOutputDescriptor):
    def __init__(self):
        super().__init__()

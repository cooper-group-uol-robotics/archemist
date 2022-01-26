from bson.objectid import ObjectId
from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from archemist.state.material import Liquid
from archemist.exceptions.exception import InvalidLiquidError

class PeristalticLiquidDispensing(Station):
    def __init__(self, db_name: str, station_dict: dict, liquids: list, solids: list):

        if len(station_dict) > 1:
            parameters = station_dict.pop('parameters')
            station_dict['pump_liquid_map'] = dict()
            for _, pumpId in parameters['liquid_pump_map'].items():
                for liquid in liquids:
                    if liquid.pump_id == pumpId:
                        station_dict['pump_liquid_map'].update({pumpId: liquid.object_id})
        
        super().__init__(db_name,station_dict)

    @classmethod
    def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
        return cls(db_name, station_dict, liquids, solids)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        station_dict = {'object_id':object_id}
        return cls(db_name, station_dict, None, None)

    def get_liquid(self, pumpId: str):
        liquid_obj_id = self.get_nested_field(f'pump_liquid_map.{pumpId}')
        return Liquid.from_object_id(self.db_name, liquid_obj_id)

    def get_pump_id(self, liquid_name: str):
        # assuming only single liquid per pump
        pump_liquid_map = self.get_field('pump_liquid_map')
        for pumpId, liquid_obj_id in pump_liquid_map.items():
            if liquid_name == Liquid.from_object_id(self.db_name, liquid_obj_id).name:
                return pumpId
            else:
                raise InvalidLiquidError(self.__class__.__name__)

    def add_to_pump_liquid_level(self, pumpId: str, added_value: float):
        modified_liquid = self.get_liquid(pumpId)
        modified_liquid.volume = modified_liquid.volume + added_value

    def add_liquid(self, liquid_nam: str, added_volume: float):
        pumpId = self.get_pump_id(liquid_nam)
        self.add_to_pump_liquid_level(pumpId, added_volume)

    def reduce_pump_liquid_level(self, pumpId: str, value: float):
        modified_liquid = self.get_liquid(pumpId)
        modified_liquid.volume = modified_liquid.volume - value

    def dispense_liquid(self, liquid_nam: str, dispensed_volume: float):
        pumpId = self.get_pump_id(liquid_nam)
        self.reduce_pump_liquid_level(pumpId, dispensed_volume)

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

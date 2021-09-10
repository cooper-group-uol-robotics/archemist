from archemist.state.station import Station, Location, StationOpDescriptor, StationOutputDescriptor
from archemist.state.material import Liquid
from archemist.exceptions.exception import InvalidLiquidError


class PeristalticLiquidDispensing(Station):
    def __init__(self, id: int, loc: Location, parameters: dict, liquids: list, solids: list):
        super().__init__(id, loc)
        self._pumpLiquidMap = dict()
        for liquidName, pumpId in parameters['liquid_pump_map'].items():
            for liquid in liquids:
                if liquid.pump_id == pumpId:
                    self._pumpLiquidMap[pumpId] = liquid

    def getPumpLiquidLevel(self, pumpId: str):
        return self._pumpLiquidMap[pumpId].volume

    def getPumpID(self, requested_liquid: Liquid):
        # assuming only single liquid per pump
        for pumpId, liquid in self._pumpLiquidMap.items():
            if liquid.name == requested_liquid.name:
                return pumpId
            else:
                raise InvalidLiquidError(self.__class__.__name__)

    def getLiquidLevel(self, liquid: Liquid):
        pumpId = self.getPumpID(liquid)
        return self.getPumpLiquidLevel(pumpId)

    def addToPumpLiquidLevel(self, pumpId: str, added_value: float):
        self._pumpLiquidMap[pumpId].volume = self._pumpLiquidMap[pumpId].volume + added_value

    def addLiquid(self, added_liquid: Liquid):
        pumpId = self.getPumpID(added_liquid)
        self.addToPumpLiquidLevel(pumpId, added_liquid.volume)

    def reducePumpLiquidLevel(self, pumpId: str, added_value: float):
        self._pumpLiquidMap[pumpId].volume = self._pumpLiquidMap[pumpId].volume - added_value

    def dispenseLiquid(self, dispensed_liquid: Liquid):
        pumpId = self.getPumpID(dispensed_liquid)
        self.reducePumpLiquidLevel(pumpId, dispensed_liquid.volume)

    def setStationOp(self, stationOp: StationOpDescriptor):
        if (stationOp.stationName == self.__class__.__name__):
            self.dispenseLiquid(stationOp.liquid)
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
    def __init__(self, opName: str):
        super().__init__(opName=opName)

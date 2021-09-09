from archemist.state.station import Station, Location, StationOpDescriptor, StationOutputDescriptor
from archemist.state.material import Liquid
from archemist.exceptions.exception import InvalidLiquidError


class PeristalticLiquidDispensing(Station):
    def __init__(self, id: int, loc: Location, pumpLiquidMap: dict):
        super().__init__(id, loc)
        self._pumpLiquidMap = pumpLiquidMap

    def getPumpLiquidLevel(self, pumpId: str):
        return self._pumpLiquidMap[pumpId].volume

    def getPumpID(self, requested_liquid: Liquid):
        # assuming only single liquid per pump
        for pumpId, liquid in _pumpLiquidMap.items():
            if liquid == requested_liquid:
                return pumpId
            else:
                raise InvalidLiquidError(self.__class__.__name__)

    def getLiquidLevel(self, liquid: Liquid):
        pumpId = self.getPumpID(liquid)
        return getPumpLiquidLevel(pumpId)

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
        if (stationOp.stationName == self.__class__):
            self.dispenseLiquid(stationOp.liquid, sstationOp.liquid.volume)
        else:
            raise ValueError

''' ==== Station Operation Descriptors ==== '''

class PeristalticPumpOpDescriptor(StationOpDescriptor):
    def __init__(self, liquid: Liquid):
        super().__init__(stationName=PeristalticLiquidDispensing.__class__)
        self._liquid = liquid

    @property
    def liquid(self):
        return self._liquid



''' ==== Station Output Descriptors ==== '''

class PeristalticPumpOutputDescriptor(StationOutputDescriptor):
    def __init__(self, opName: str, success:bool):
        super().__init__(opName=opName, succes=success)

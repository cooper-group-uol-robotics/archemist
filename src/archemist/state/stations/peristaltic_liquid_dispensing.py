from archemist.state.station import Station, Location, StationOpDescriptor, StationOutputDescriptor
from archemist.state.material import Liquid


class PeristalticLiquidDispensing(Station):
    def __init__(self, name: str, id: int, loc: Location, pumpLiquidLevels: dict, liquidsPumpMap: dict):
        super().__init__(name, id, loc)
        self._pumpLiquidLevels = pumpLiquidLevels
        self._liquidsPumpMap = liquidsPumpMap

    def getPumpLiquidLevel(self, pumpId: int):
        return self._pumpLiquidLevels[pumpId]

    def getLiquidLevel(self, liquid: Liquid):
        return self._pumpLiquidLevels[_liquidsPumpMap[liquid]]

    def addToPumpLiquidLevel(self, pumpId: int, added_value: float):
        self._pumpLiquidLevels[pumpId] = self._pumpLiquidLevels[pumpId] + added_value

    def addLiquid(self, liquid: Liquid, added_value: float):
        self._pumpLiquidLevels[_liquidsPumpMap[liquid]] = self.getLiquidLevel(liquid) + added_value

    def reducePumpLiquidLevel(self, pumpId: int, added_value: float):
        self._pumpLiquidLevels[pumpId] = self._pumpLiquidLevels[pumpId] - added_value

    def reduceLiquid(self, liquid: Liquid, added_value: float):
        self._pumpLiquidLevels[_liquidsPumpMap[liquid]] = self.getLiquidLevel(liquid) - added_value

    def setStationOp(self, stationOp: StationOpDescriptor):
        if (stationOp.stationName == self.__class__):
            self.reduceLiquid(stationOp.liquid, stationOp.amountToDispense)
        else:
            raise ValueError

''' ==== Station Operation Descriptors ==== '''

class PeristalticPumpOpDescriptor(StationOpDescriptor):
    def __init__(self, liquid: Liquid, amountToDispense: float):
        super().__init__(stationName=PeristalticLiquidDispensing.__class__)
        self._liquid = liquid
        self._amountToDispense = amountToDispense

    @property
    def liquid(self):
        return self._liquid

    @property
    def amountToDispense(self):
        return self._amountToDispense


''' ==== Station Output Descriptors ==== '''

class PeristalticPumpOutputDescriptor(StationOutputDescriptor):
    def __init__(self, opName: str, success:bool):
        super().__init__(opName=opName, succes=success)

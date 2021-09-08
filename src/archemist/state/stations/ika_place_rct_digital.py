from src.archemist.state.station import Station, Location
from src.archemist.util.rosMsgCoder import rosMsgCoder
from enum import Enum

class ikaState(Enum):
    STOPPED = 0
    HEATING = 1
    STIRRING = 2
    HEATINGSTIRRING = 3

class ika_plate_rct_digital(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
        self._currentTemperature = None
        self._setTemperature = None
        self._stirringSpeed = None
        self._externalTemp = None
        self._viscosityTrend = None
        self._state = ikaState.STOPPED

    def readDescriptor(self, descriptor):
        self._currentTemperature = descriptor[0]
        self._setTemperature = descriptor[1]
        self._stirringSpeed = descriptor[2]
        self._externalTemp = descriptor[3]
        self._viscosityTrend = descriptor[4]
        self._state = descriptor[5]

class ikaOpDescriptor(StationOpDescriptor):
    def __init__(self):
        super().__init__()

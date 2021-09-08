from src.archemist.state.station import Station, Location
from src.archemist.util.rosMsgCoder import rosMsgCoder
from enum import Enum


class ikaMode(Enum):
    HEATING = 1
    STIRRING = 2
    HEATINGSTIRRING = 3


class ika_plate_rct_digital(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
        self._currentTemperature = None
        self._setTemperature = None
        self._currentStirringSpeed = None
        self._setStirringSpeed = None
        self._externalTemp = None
        self._viscosityTrend = None
        self._mode = None
        self._executing = False

    @property
    def currentTemperature(self):
        return self._currentTemperature

    @currentTemperature.setter
    def currentTemperature(self, value):
        self._currentTemperature = value

    @property
    def setTemperature(self):
        return self._setTemperature

    @setTemperature.setter
    def setTemperature(self, value):
        self._setTemperature = value

    @property
    def currentStirringSpeed(self):
        return self._currentStirringSpeed

    @currentStirringSpeed.setter
    def currentStirringSpeed(self, value):
        self._currentStirringSpeed = value

    @property
    def setStirringSpeed(self):
        return self._setStirringSpeed

    @setStirringSpeed.setter
    def setStirringSpeed(self, value):
        self._setStirringSpeed = value

    @property
    def externalTemp(self):
        return self._externalTemp

    @externalTemp.setter
    def externalTemp(self, value):
        self._externalTemp = value

    @property
    def viscosityTrend(self):
        return self._viscosityTrend

    @viscosityTrend.setter
    def viscosityTrend(self, value):
        self._viscosityTrend = value

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        if isinstance(value, ikaMode):
            self._mode = value
        else:
            raise ValueError

    @property
    def executing(self):
        return self._executing

    @executing.setter
    def executing(self, value):
        self._executing = value

    def readDescriptor(self, descriptor):
        self._currentTemperature = descriptor[0]
        self._setTemperature = descriptor[1]
        self._stirringSpeed = descriptor[2]
        self._externalTemp = descriptor[3]
        self._viscosityTrend = descriptor[4]

class ikaOpDescriptor(StationOpDescriptor):
    def __init__(self):
        super().__init__()

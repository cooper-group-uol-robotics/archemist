from src.archemist.state.station import Station, Location
from enum import Enum


class IKAMode(Enum):
    HEATING = 1
    STIRRING = 2
    HEATINGSTIRRING = 3

''' ==== Station Description ==== '''

class IkaPlateRCTDigital(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(id, loc)
        self._mode = None
        self._currentTemperature = None
        self._setTemperature = None
        self._currentStirringSpeed = None
        self._setStirringSpeed = None
        self._externalTemp = None
        self._viscosityTrend = None
        self._setDuration = None

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
    def setDuration(self):
        return self._setDuration

    @setDuration.setter
    def setDuration(self, value):
        self._setDuration = value

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
        if isinstance(value, IKAMode):
            self._mode = value
        else:
            raise ValueError

    def setStationOp(self, stationOp: StationOpDescriptor):
        if (stationOp.stationName == self.__class__):
            self._stationOp = stationOp
            self._mode = stationOp.mode
            self._setTemperature = stationOp.setTemperature
            self._setStirringSpeed = stationOp.setStirringSpeed
            self._setDuration = stationOp.duration
        else:
            raise ValueError


''' ==== Station Operation Descriptors ==== '''

class IKAHeatingStriingOpDescriptor(StationOpDescriptor):
    def __init__(self, setTemperature: int, setStirringSpeed: int, duration: int):
        super().__init__(stationName=IkaPlateRCTDigital.__class__)
        self._mode = IKAMode.HEATINGSTIRRING
        self._setTemperature = setTemperature
        self._setStirringSpeed = setStirringSpeed
        self._duration = duration

    @property
    def setTemperature(self):
        return self._setTemperature

    @property
    def setStirringSpeed(self):
        return self._setStirringSpeed

    @property
    def mode(self):
        return self._mode

    @property
    def duration(self):
        return self._duration

class IKAHeatingOpDescriptor(StationOpDescriptor):
    def __init__(self, setTemperature: int, duration: int):
        super().__init__(stationName=IkaPlateRCTDigital.__class__)
        self._mode = IKAMode.HEATING
        self._setTemperature = setTemperature
        self._setStirringSpeed = 0
        self._duration = duration

    @property
    def setTemperature(self):
        return self._setTemperature


    @property
    def duration(self):
        return self._duration

    @property
    def mode(self):
        return self._mode

class IKAStriingOpDescriptor(StationOpDescriptor):
    def __init__(self, setStirringSpeed: int, duration: int):
        super().__init__(stationName=IkaPlateRCTDigital.__class__)
        self._mode = IKAMode.STIRRING
        self._setTemperature = 0
        self._setStirringSpeed = setStirringSpeed
        self._duration = duration

    @property
    def setStirringSpeed(self):
        return self._setStirringSpeed

    @property
    def mode(self):
        return self._mode

    @property
    def duration(self):
        return self._duration

''' ==== Station Output Descriptors ==== '''

class IKAOutputDescriptor(StationOutputDescriptor):
    def __init__(self, opName: str, success:bool):
        super().__init__(opName=opName, succes=success)






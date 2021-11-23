from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from enum import Enum

from archemist.util.location import Location


class IKAMode(Enum):
    HEATING = 1
    STIRRING = 2
    HEATINGSTIRRING = 3

''' ==== Station Description ==== '''

class IkaPlateRCTDigital(Station):
    def __init__(self, id: int, location: Location,
                 parameters: dict, liquids: list, solids: list):
        super().__init__(id, location)
        self._mode = None
        self._current_temperature = None
        self._set_temperature = None
        self._current_stirring_speed = None
        self._set_stirring_speed = None
        self._external_temp = None
        self._viscosity_trend = None
        self._set_duration = None

    @property
    def current_temperature(self):
        return self._current_temperature

    @current_temperature.setter
    def current_temperature(self, value):
        self._current_temperature = value

    @property
    def set_temperature(self):
        return self._set_temperature

    @set_temperature.setter
    def set_temperature(self, value):
        self._set_temperature = value

    @property
    def current_stirring_speed(self):
        return self._current_stirring_speed

    @current_stirring_speed.setter
    def current_stirring_speed(self, value):
        self._current_stirring_speed = value

    @property
    def set_stirring_speed(self):
        return self._set_stirring_speed

    @set_stirring_speed.setter
    def set_stirring_speed(self, value):
        self._set_stirring_speed = value

    @property
    def set_duration(self):
        return self._set_duration

    @set_duration.setter
    def set_duration(self, value):
        self._set_duration = value

    @property
    def external_temp(self):
        return self._external_temp

    @external_temp.setter
    def external_temp(self, value):
        self._external_temp = value

    @property
    def viscosity_trend(self):
        return self._viscosity_trend

    @viscosity_trend.setter
    def viscosity_trend(self, value):
        self._viscosity_trend = value

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
        if (stationOp.stationName == self.__class__.__name__):
            self._stationOp = stationOp
            self._mode = stationOp.mode
            self._set_temperature = stationOp.set_temperature
            self._set_stirring_speed = stationOp.set_stirring_speed
            self._set_duration = stationOp.duration
        else:
            raise ValueError


''' ==== Station Operation Descriptors ==== '''

class IKAHeatingStirringOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=IkaPlateRCTDigital.__class__.__name__, output=output)
        self._mode = IKAMode.HEATINGSTIRRING
        self._set_temperature = properties['temperature']
        self._set_stirring_speed = properties['rpm']
        self._duration = properties['duration']

    @property
    def set_temperature(self):
        return self._set_temperature

    @property
    def set_stirring_speed(self):
        return self._set_stirring_speed

    @property
    def mode(self):
        return self._mode

    @property
    def duration(self):
        return self._duration

class IKAHeatingOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=IkaPlateRCTDigital.__class__.__name__, output=output)
        self._mode = IKAMode.HEATING
        self._set_temperature = properties['temperature']
        self._set_stirring_speed = 0
        self._duration = properties['duration']

    @property
    def set_temperature(self):
        return self._set_temperature


    @property
    def duration(self):
        return self._duration

    @property
    def mode(self):
        return self._mode

class IKAStirringOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=IkaPlateRCTDigital.__class__.__name__, output=output)
        self._mode = IKAMode.STIRRING
        self._set_temperature = 0
        self._set_stirring_speed = properties['rpm']
        self._duration = properties['duration']

    @property
    def set_stirring_speed(self):
        return self._set_stirring_speed

    @property
    def mode(self):
        return self._mode

    @property
    def duration(self):
        return self._duration

''' ==== Station Output Descriptors ==== '''

class IKAOutputDescriptor(StationOutputDescriptor):
    def __init__(self):
        super().__init__()

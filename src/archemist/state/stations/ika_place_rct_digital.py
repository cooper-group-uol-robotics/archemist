from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from enum import Enum
from bson.objectid import ObjectId

class IKAMode(Enum):
    HEATING = 1
    STIRRING = 2
    HEATINGSTIRRING = 3

''' ==== Station Description ==== '''

class IkaPlateRCTDigital(Station):
    def __init__(self, db_name: str, station_dict: dict, liquids: list, solids: list):
        if len(station_dict) > 1:
            station_dict['mode'] = None
            station_dict['current_temperature'] = None
            station_dict['set_temperature'] = None
            station_dict['current_stirring_speed'] = None
            station_dict['set_stirring_speed'] = None
            station_dict['external_temp'] = None
            station_dict['viscosity_trend'] = None
            station_dict['set_duration'] = None

        super().__init__(db_name,station_dict)

    @classmethod
    def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
        return cls(db_name, station_dict, liquids, solids)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        station_dict = {'object_id':object_id}
        return cls(db_name, station_dict, None, None)

    @property
    def current_temperature(self):
        return self.get_field('current_temperature')

    @current_temperature.setter
    def current_temperature(self, value):
        self.update_field('current_temperature', value)

    @property
    def set_temperature(self):
        return self.get_field('set_temperature')

    @set_temperature.setter
    def set_temperature(self, value):
        self.update_field('set_temperature', value)

    @property
    def current_stirring_speed(self):
        return self.get_field('current_stirring_speed')

    @current_stirring_speed.setter
    def current_stirring_speed(self, value):
        self.update_field('current_stirring_speed', value)

    @property
    def set_stirring_speed(self):
        return self.get_field('set_stirring_speed')

    @set_stirring_speed.setter
    def set_stirring_speed(self, value):
        self.update_field('set_stirring_speed', value)

    @property
    def set_duration(self):
        return self.get_field('set_duration')

    @set_duration.setter
    def set_duration(self, value):
        self.update_field('set_duration', value)

    @property
    def external_temp(self):
        return self.get_field('external_temp')

    @external_temp.setter
    def external_temp(self, value):
        self.update_field('external_temp', value)

    @property
    def viscosity_trend(self):
        return self.get_field('viscosity_trend')

    @viscosity_trend.setter
    def viscosity_trend(self, value):
        self.update_field('viscosity_trend', value)

    @property
    def mode(self):
        return IKAMode(self.get_field('mode'))

    @mode.setter
    def mode(self, mode):
        if isinstance(mode, IKAMode):
            self.update_field('mode', mode.value)
        else:
            raise ValueError

    def set_station_op(self, stationOp: StationOpDescriptor):
        self.mode = stationOp.mode
        self.set_temperature = stationOp.set_temperature
        self.set_stirring_speed = stationOp.set_stirring_speed
        self.set_duration = stationOp.duration
        super().set_station_op(stationOp)

    def finish_station_operation(self):
        self.set_temperature = None
        self.set_stirring_speed = None
        self.set_duration = None
        super().finish_station_operation()


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

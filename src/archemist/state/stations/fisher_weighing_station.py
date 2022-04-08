from bson.objectid import ObjectId
from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor

''' ==== Station Description ==== '''
class FisherWeightingStation(Station):
    def __init__(self, db_name: str, station_dict: dict, liquids: list, solids: list):
        if len(station_dict) > 1:
            station_dict.pop('parameters')
        super().__init__(db_name, station_dict)

    @classmethod
    def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
        return cls(db_name, station_dict, liquids, solids)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        station_dict = {'object_id':object_id}
        return cls(db_name, station_dict, None, None)

''' ==== Station Operation Descriptors ==== '''

class FisherWeightNowpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=FisherWeightingStation.__class__.__name__, output=output)

class FisherWeightStablepDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=FisherWeightingStation.__class__, output=output)


''' ==== Station Output Descriptors ==== '''

class FisherOutputDescriptor(StationOutputDescriptor):
    def __init__(self):
        super().__init__()
        self._weight = -1

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, value):
        self._weight = value

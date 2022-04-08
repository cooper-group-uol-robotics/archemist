from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from bson.objectid import ObjectId


''' ==== Station Description ==== '''
class SolubilityStation(Station):
    def __init__(self, db_name: str, station_dict: dict, liquids: list, solids: list):
        if len(station_dict) > 1:
            station_dict.pop('parameters')
            station_dict['recording'] = False
        super().__init__(db_name, station_dict)

    @classmethod
    def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
        return cls(db_name, station_dict, liquids, solids)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        station_dict = {'object_id':object_id}
        return cls(db_name, station_dict, None, None)

    @property
    def recording(self):
        return self.get_field('recording')

    @recording.setter
    def recording(self, value):
        if (isinstance(value, bool)):
            self.update_field('recording', value)
        else:
            raise ValueError

''' ==== Station Operation Descriptors ==== '''

class SolubilityOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=SolubilityStation.__class__, output=output)
        self._duration = properties['duration']
        

    @property
    def duration(self):
        return self._duration

''' ==== Station Output Descriptors ==== '''

class SolubilityDescriptor(StationOutputDescriptor):
    def __init__(self):
        super().__init__()
        self._turbidity = -1

    @property
    def turbidity(self):
        return self._turbidity

    @turbidity.setter
    def turbidity(self,value):
        self._turbidity = value

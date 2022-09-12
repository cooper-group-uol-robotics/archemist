from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from bson.objectid import ObjectId


''' ==== Station Description ==== '''
class LightBoxStation(Station):
    def __init__(self, db_name: str, station_dict: dict, liquids: list, solids: list):
        super().__init__(db_name, station_dict)

    @classmethod
    def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
        return cls(db_name, station_dict, liquids, solids)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        station_dict = {'object_id':object_id}
        return cls(db_name, station_dict, None, None)

''' ==== Station Operation Descriptors ==== '''

class VialProcessingOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=LightBoxStation.__class__, output=output)
      
        
''' ==== Station Output Descriptors ==== '''

class ColourDescriptor(StationOutputDescriptor):
    def __init__(self):
        super().__init__()
        self._filename = ''
        self._red_value = -1
        self._green_value = -1
        self._blue_value = -1

    @property
    def file_name(self):
        return self._file_name

    @file_name.setter
    def file_name(self,value):
        self._file_name = value

    @property
    def red_value(self):
        return self._red_value

    @red_value.setter
    def red_value(self,value):
        self._red_value = value

    @property
    def green_value(self):
        return self._green_value

    @red_value.setter
    def green_value(self,value):
        self._green_value = value

    @property
    def blue_value(self):
        return self._blue_value

    @blue_value.setter
    def blue_value(self,value):
        self._blue_value = value
    
    

    

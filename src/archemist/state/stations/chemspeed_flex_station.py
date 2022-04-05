from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from enum import Enum
from bson.objectid import ObjectId

class ChemSpeedStatus(Enum):
    DOORS_OPEN = 0
    DOORS_CLOSED = 1
    RUNNING_JOB = 2
    JOB_COMPLETE = 3

''' ==== Station Description ==== '''

class ChemSpeedFlexStation(Station):
    def __init__(self, db_name: str, station_dict: dict, liquids: list, solids: list):
        if len(station_dict) > 1:
            station_dict['status'] = None

        super().__init__(db_name,station_dict)

    @classmethod
    def from_dict(cls, db_name: str, station_dict: dict, liquids: list, solids: list):
        return cls(db_name, station_dict, liquids, solids)

    @classmethod
    def from_object_id(cls, db_name: str, object_id: ObjectId):
        station_dict = {'object_id':object_id}
        return cls(db_name, station_dict, None, None)

    @property
    def status(self):
        return ChemSpeedStatus(self.get_field('status'))

    @status.setter
    def status(self, status):
        if isinstance(status, ChemSpeedStatus):
            self.update_field('status', status.value)
        else:
            raise ValueError


''' ==== Station Operation Descriptors ==== '''

class CSOpenDoorOpDescriptor(StationOpDescriptor):
    def __init__(self):
        output = StationOutputDescriptor()
        super().__init__(stationName=ChemSpeedFlexStation.__class__.__name__, output=output)

class CSCloseDoorOpDescriptor(StationOpDescriptor):
    def __init__(self):
        output = StationOutputDescriptor()
        super().__init__(stationName=ChemSpeedFlexStation.__class__.__name__, output=output)

class CSStartJobOpDescriptor(StationOpDescriptor):
    def __init__(self):
        output = StationOutputDescriptor()
        super().__init__(stationName=ChemSpeedFlexStation.__class__.__name__, output=output)

''' ==== Station Output Descriptors ==== '''

class CSJobOutputDescriptor(StationOutputDescriptor):
    def __init__(self):
        self._results_file_name = ''
        super().__init__()

    @property
    def results_file_name(self):
        return self._results_file_name

    @results_file_name.setter
    def results_file_name(self, file_name):
        self._results_file_name = file_name


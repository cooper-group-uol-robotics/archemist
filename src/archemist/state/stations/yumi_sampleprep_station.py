from archemist.state.station import Station, Location, StationOpDescriptor, StationOutputDescriptor
from enum import Enum
from bson.objectid import ObjectId

class yumi_sampleprep_stationStatus(Enum):
    LOAD_STIR = 0
    LOAD_SHKR = 1
    LOAD_PLATE = 2
    CAPS = 3

''' ==== Station Description ==== '''

class yumi_sampleprep_station(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(id, loc)

    @property
    def status(self):
        return yumi_sampleprep_stationStatus(self.get_field('status'))



''' ==== Station Operation Descriptors ==== '''

class YSLoadStirOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        super().__init__(stationName=yumi_sampleprep_station.__class__.__name__, output=output)

class YSLoadShkrOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        output = StationOutputDescriptor()
        super().__init__(stationName=yumi_sampleprep_station.__class__.__name__, output=output)

class YSLoadPlateOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        output = StationOutputDescriptor()
        super().__init__(stationName=yumi_sampleprep_station.__class__.__name__, output=output)

class YSCapsOpDescriptor(StationOpDescriptor):
    def __init__(self, properties: dict, output: StationOutputDescriptor):
        output = StationOutputDescriptor()
        super().__init__(stationName=yumi_sampleprep_station.__class__.__name__, output=output)




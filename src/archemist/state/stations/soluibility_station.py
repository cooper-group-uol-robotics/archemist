from archemist.state.station import Station, StationOpDescriptor, StationOutputDescriptor
from archemist.util.location import Location


''' ==== Station Description ==== '''
class SolubilityStation(Station):
    def __init__(self, id: int, location: Location,
                 parameters: dict, liquids: list, solids: list):
        super().__init__(id, location)
        self._recording = False

    @property
    def recording(self):
        return self._recording

    @recording.setter
    def recording(self, value):
        if (isinstance(value, bool)):
            self._recording = value
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

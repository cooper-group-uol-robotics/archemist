from archemist.state.station import Station, Location, StationOpDescriptor, StationOutputDescriptor


''' ==== Station Description ==== '''
class SolubilityStation(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(id, loc)
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
    def __init__(self, duration: int):
        super().__init__(stationName=SolubilityStation.__class__)
        self._duration = duration
        

    @property
    def duration(self):
        return self._duration

''' ==== Station Output Descriptors ==== '''

class SolubilityDescriptor(StationOutputDescriptor):
    def __init__(self, opName: str, turbidity: float, success:bool):
        super().__init__(opName=opName, succes=success)
        self._turbidity = turbidity

    @property
    def turbidity(self):
        return self._turbidity

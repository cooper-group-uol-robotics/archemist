from archemist.state.station import Station, Location, StationOpDescriptor, StationOutputDescriptor


''' ==== Station Description ==== '''
class FisherWeightingStation(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(id, loc)

''' ==== Station Operation Descriptors ==== '''

class FisherWeightNowpDescriptor(StationOpDescriptor):
    def __init__(self):
        super().__init__(stationName=FisherWeightingStation.__class__.__name__)

class FisherWeightStablepDescriptor(StationOpDescriptor):
    def __init__(self):
        super().__init__(stationName=FisherWeightingStation.__class__)


''' ==== Station Output Descriptors ==== '''

class CameraOutputDescriptor(StationOutputDescriptor):
    def __init__(self, opName: str, weight: float, success:bool):
        super().__init__(opName=opName, succes=success)
        self._weight = weight

    @property
    def weight(self):
        return self._weight

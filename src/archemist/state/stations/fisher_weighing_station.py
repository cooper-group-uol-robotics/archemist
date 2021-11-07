from transitions.core import Machine
from archemist.state.station import Station, Location, StationOpDescriptor, StationOutputDescriptor


''' ==== Station Description ==== '''
class FisherWeightingStation(Station):
    def __init__(self, id: int, process_sm: Machine, parameters: dict, 
                 liquids: list, solids: list):
        super().__init__(id, process_sm)

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

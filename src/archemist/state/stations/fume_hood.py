from archemist.state.station import Station
from archemist.util import Location


class fume_hood(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(id, loc)

from archemist.state.station import Station, Location


class pxrd_analyser(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(id, loc)

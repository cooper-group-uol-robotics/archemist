from src.archemist.state.station import Station, Location


class pxrd_analyser(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
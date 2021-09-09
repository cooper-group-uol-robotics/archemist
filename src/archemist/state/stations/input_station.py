from archemist.state.station import Station, Location


class InputStation(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)

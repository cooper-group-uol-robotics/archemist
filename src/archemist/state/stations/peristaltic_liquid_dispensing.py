from src.archemist.state.station import Station, Location


class PeristalticLiquidDispensing(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(name, id, loc)
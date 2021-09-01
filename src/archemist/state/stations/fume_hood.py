from src.archemist.state.station import AbstractStation, Location


class fume_hood(AbstractStation):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
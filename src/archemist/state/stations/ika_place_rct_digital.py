from src.archemist.state.station import Station, Location


class ika_plate_rct_digital(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
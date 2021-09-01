from src.archemist.state.station import HeaterStirrerStation, Location


class ika_plate_rct_digital(HeaterStirrerStation):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
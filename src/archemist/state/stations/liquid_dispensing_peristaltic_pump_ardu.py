from src.archemist.state.station import Station, Location


class liquid_dispensing_peristaltic_pump_ardu(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
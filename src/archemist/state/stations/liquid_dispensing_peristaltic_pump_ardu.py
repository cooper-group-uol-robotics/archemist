from src.archemist.state.station import LiquidDispensingStation, Location


class liquid_dispensing_peristaltic_pump_ardu(LiquidDispensingStation):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
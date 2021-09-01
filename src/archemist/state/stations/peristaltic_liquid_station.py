from src.archemist.state.station import LiquidDispensingStation, Location


class peristalticLiquidDispenser(LiquidDispensingStation):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
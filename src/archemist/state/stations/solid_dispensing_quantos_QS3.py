from src.archemist.state.station import Station, Location


class solid_dispensing_quantos_QS3(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
from archemist.state.station import Station, Location


class SolidDispensingQuantosQS2(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
        self.carouselPos = 0

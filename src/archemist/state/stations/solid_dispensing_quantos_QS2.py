from archemist.state.station import Station, Location


class SolidDispensingQuantosQS2(Station):
    def __init__(self, id: int, loc: Location):
        super().__init__(id, loc)
        self.carouselPos = 0

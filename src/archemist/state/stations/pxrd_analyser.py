from src.archemist.state.station import CrystalAnalysisStation, Location


class pxrd_analyser(CrystalAnalysisStation):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)
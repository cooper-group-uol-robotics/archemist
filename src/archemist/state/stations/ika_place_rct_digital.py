from src.archemist.state.station import Station, Location
from src.archemist.util.rosMsgCoder import rosMsgCoder

class ika_plate_rct_digital(Station):
    def __init__(self, name: str, id: int, loc: Location):
        super().__init__(name, id, loc)

class ikaOpDescriptor(StationOpDescriptor):
    def __init__(self):
        super().__init__()

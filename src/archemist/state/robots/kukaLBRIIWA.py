from src.archemist.state.robot import mobileRobot
from src.archemist.state.station import Location


class kukaLBRIIWA(mobileRobot):
    def __init__(self, name: str, id: int, initLocation: Location):
        super().__init__(name, id, initLocation)
    
    def moveToLocation(self, loc: Location):
        #moving location code
        self._location = loc
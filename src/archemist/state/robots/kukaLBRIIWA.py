from src.archemist.state.robot import mobileRobot
from src.archemist.state.station import Location


class kukaLBRIIWA(mobileRobot):
    def __init__(self, name: str, id: int):
        super().__init__(name, id)
    
    def moveToLocation(self, loc: Location):
        #moving location code
        self._location = loc
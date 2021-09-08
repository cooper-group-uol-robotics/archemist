from archemist.state.robot import mobileRobot
from archemist.state.station import Location


class KukaLBRIIWA(mobileRobot):
    def __init__(self, name: str, id: int):
        super().__init__(name, id)
    
    def moveToLocation(self, loc: Location):
        #moving location code
        self._location = loc
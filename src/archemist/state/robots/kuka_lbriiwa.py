from archemist.state.robot import robot
from archemist.state.station import Location


class kukaLBRIIWA(robot):
    def __init__(self, name: str, id: int, initLocation: Location):
        super().__init__(name, id, initLocation)
    
    def moveToLocation(self, loc: Location):
        #moving location code
        self._location = loc
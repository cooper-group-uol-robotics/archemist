from src.archemist.state.station import Location


class robot:
    def __init__(self, name: str, id: int):
        self._name = name
        self._id = id
        self._available = False
        self._operational = False
    
    @property
    def available(self):
        return self._available

    @available.setter
    def available(self, value):
        if isinstance(value, bool):
            self._available = value
        else:
            raise ValueError

    @property
    def operational(self):
        return self._operational

    @operational.setter
    def operational(self, value):
        if isinstance(value, bool):
            self._operational = value
        else:
            raise ValueError


class mobileRobot(robot):
    def __init__(self, name: str, id: int):
        super().__init__(name, id)
        self._location = None

class armRobot(robot):
    def __init__(self, name: str, id: int):
        super().__init__(name, id)
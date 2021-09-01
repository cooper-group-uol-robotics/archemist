class robot:
    def __init__(self, name: str, id: int):
        self._name = name
        self._id = id

class mobileRobot(robot):
    def __init__(self, name: str, id: int):
        super().__init__(name, id)

class armRobot(robot):
    def __init__(self, name: str, id: int):
        super().__init__(name, id)
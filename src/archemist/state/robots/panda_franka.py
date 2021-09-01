from archemist.state.robot import robot


class pandaFranka(robot):
    def __init__(self, name: str, id: int):
        super().__init__(name, id)
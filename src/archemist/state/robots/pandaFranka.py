from src.archemist.state.robot import armRobot


class pandaFranka(armRobot):
    def __init__(self, name: str, id: int):
        super().__init__(name, id)
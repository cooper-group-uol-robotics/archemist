from src.archemist.state.robot import armRobot


class pandaFranka(armRobot):
    def __init__(self, id: int):
        super().__init__(id)
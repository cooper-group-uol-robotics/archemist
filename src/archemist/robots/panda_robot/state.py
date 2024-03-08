from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.models.robot_model import RobotModel
from archemist.core.state.robot import FixedRobot
from typing import Union


class PandaRobot(FixedRobot):
    def __init__(self, robot_model: Union[RobotModel, ModelProxy]) -> None:
        super().__init__(robot_model)

from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.models.robot_model import RobotModel
from archemist.core.state.robot import Robot
from typing import Union

class YuMiRobot(Robot):
    def __init__(self, robot_model: Union[RobotModel, ModelProxy]) -> None:
        super().__init__(robot_model)

    @classmethod
    def from_dict(cls, robot_document: dict):
        model = RobotModel()
        cls._set_model_common_fields(model, robot_document)
        model.save()
        return cls(model)
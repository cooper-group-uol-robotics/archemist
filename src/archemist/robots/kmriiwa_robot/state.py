from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.models.robot_model import MobileRobotModel
from archemist.core.state.robot import MobileRobot
from typing import Union
    
class KMRIIWARobot(MobileRobot):
    def __init__(self, robot_model: Union[MobileRobotModel, ModelProxy]) -> None:
        super().__init__(robot_model)

    @classmethod
    def from_dict(cls, robot_document: dict):
        model = MobileRobotModel()
        cls._set_model_common_fields(model, robot_document)
        model.save()
        return cls(model)

# External
from bson.objectid import ObjectId
from typing import List


# Core
from archemist.core.models.robot_model import RobotModel
from archemist.core.models.robot_op_model import (
    RobotTaskOpDescriptorModel,
    RobotTaskType,
)
from archemist.core.state.robot import Robot
from archemist.core.state.robot_op import RobotTaskOpDescriptor
from archemist.core.util.location import Location


class YuMiRobotTask(RobotTaskOpDescriptor):
    def __init__(self, op_model: RobotTaskOpDescriptorModel):
        super().__init__(op_model)

    @classmethod
    def from_args(
        cls,
        name: str,
        type: RobotTaskType = RobotTaskType.MANIPULATION,
        params: List[str] = [],
        location: Location = Location(),
        origin_station: ObjectId = None,
        related_batch_id: int = None,
    ):
        model = (
            super()
            .from_args(name, type, params, location, origin_station, related_batch_id)
            .model
        )
        model._type = cls.__name__
        model._module = cls.__module__
        return cls(model)


class YuMiRobot(Robot):
    def __init__(self, robot_model: RobotModel) -> None:
        self._model = robot_model

    @classmethod
    def from_dict(cls, robot_document: dict):
        model = RobotModel()
        model._type = cls.__name__
        model._module = cls.__module__
        cls._set_model_common_fields(robot_document, model)
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = RobotModel.objects.get(id=object_id)
        return cls(model)
